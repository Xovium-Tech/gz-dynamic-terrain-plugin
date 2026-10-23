#include "dynamic_terrain/core/DownloadCache.hh"
#include "../src/core/DownloadRetry.hh"
#include <curl/curl.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <spawn.h>
#include <unistd.h>
extern char **environ;

namespace
{
using namespace dynamic_terrain;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
void require(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
bool valid(const std::vector<unsigned char> &bytes)
{ return bytes.size() >= 5 && std::string(bytes.begin(), bytes.begin() + 5) == "tile:"; }
size_t collect(void *data, size_t size, size_t count, void *out)
{ static_cast<std::string *>(out)->append(static_cast<char *>(data), size * count); return size * count; }
int requests(const std::string &base, const std::string &endpoint)
{
    CURL *curl = curl_easy_init(); std::string result;
    const auto url = base + "/count?path=" + endpoint;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, collect);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &result);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 3000L);
    const auto code = curl_easy_perform(curl); curl_easy_cleanup(curl);
    require(code == CURLE_OK, "fixture metrics failed"); return std::stoi(result);
}
}

int main(int argc, char **argv)
{
    try
    {
        require(argc >= 3, "usage: DownloadCache_TEST URL CACHE [--child]");
        const std::string base = argv[1]; const fs::path root = argv[2];
        DownloadCacheOptions options; options.concurrency = 2; options.perHost = 1;
        options.retries = 2; options.timeoutMs = 2000; options.maxResponseBytes = 1024;
        DownloadCache cache(options);
        auto request = [&](const std::string &endpoint, const std::string &file = "") {
            return DownloadRequest{endpoint, base + endpoint,
                root / (file.empty() ? endpoint.substr(endpoint.find_last_of('/') + 1) : file), valid};
        };
        if (argc == 4)
        {
            const auto result = cache.Ensure({request("/slow/process", "process")});
            return result.failed || result.cancelled ? 1 : 0;
        }
        fs::create_directories(root);
        const auto local = root / "local source.bin";
        { std::ofstream file(local); file << "tile:local"; }
        auto localCopy = cache.Ensure({DownloadRequest{"local", local.string(), root / "local-copy", valid}});
        auto uriCopy = cache.Ensure({DownloadRequest{"URI", "file://" + local.string(), root / "uri-copy", valid}});
        require(localCopy.downloaded == 1 && uriCopy.downloaded == 1 && localCopy.connections == 0,
                "local raster sources stopped working");
        auto first = cache.Ensure({request("/tile/first")});
        require(first.downloaded == 1 && first.connections == 1, "first transfer failed");
        auto second = cache.Ensure({request("/tile/second")});
        require(second.downloaded == 1 && second.connections == 0, "connection not reused across batches");
        std::vector<DownloadRequest> throughputRequests;
        for (int index = 0; index < 100; ++index)
            throughputRequests.push_back(request("/throughput/item" + std::to_string(index)));
        const auto throughput = cache.Ensure(throughputRequests);
        std::cout << "100 serial HTTP/1 transfers: " << throughput.elapsedSec << " seconds\n";
        require(throughput.downloaded == 100 && throughput.connections == 0 && throughput.elapsedSec < 2.0,
                "persistent serial transfers contain an avoidable scheduler delay");
        auto hit = cache.Ensure({request("/tile/first"), request("/tile/first")});
        require(hit.cached == 1 && hit.downloaded == 0, "duplicate cache hit failed");
        const auto diagnostics = cache.Diagnostics();
        require(diagnostics.cacheHits == 1 && diagnostics.downloaded == 104 &&
                diagnostics.connections == 1 && diagnostics.bytes > 500 &&
                diagnostics.queued == 0 && diagnostics.active == 0 &&
                diagnostics.bufferedBytes == 0 && diagnostics.peakActive == 1 &&
                diagnostics.peakQueued <= options.maxQueuedRequests,
                "shared download counters or idle memory accounting are incorrect");
        { std::ofstream out(root / "first", std::ios::trunc); out << "broken"; }
        auto repaired = cache.Ensure({request("/tile/first")});
        require(repaired.downloaded == 1 && requests(base, "/tile/first") == 2, "corrupt cache did not recover");
        require(cache.Ensure({request("/tile/first")}).cached == 1, "repaired file validation failed");
        const auto previousTime = fs::last_write_time(root / "first");
        const auto previousSize = fs::file_size(root / "first");
        { std::ofstream out(root / "first", std::ios::trunc); out << std::string(previousSize, 'x'); }
        fs::last_write_time(root / "first", previousTime);
        cache.InvalidateValidation(root / "first");
        require(fs::file_size(root / "first") == previousSize &&
                fs::last_write_time(root / "first") == previousTime,
                "validation invalidation modified the on-disk file");
        require(cache.Ensure({request("/tile/first")}).downloaded == 1 && requests(base, "/tile/first") == 3,
                "decoder-reported corruption with unchanged metadata did not recover");
        auto bad = cache.Ensure({request("/corrupt/body")});
        require(bad.downloaded == 1 && bad.retries == 1 && requests(base, "/corrupt/body") == 2,
                "HTTP 200 invalid body was cached");
        auto partial = cache.Ensure({request("/truncated/partial")});
        require(partial.downloaded == 1 && partial.retries == 1, "truncated response did not recover");
        auto missing = cache.Ensure({request("/permanent/missing")});
        require(missing.failed == 1 && requests(base, "/permanent/missing") == 1,
                "permanent HTTP failure retried");
        const auto retryStart = Clock::now();
        auto retry = cache.Ensure({request("/retry/recover")});
        require(retry.downloaded == 1 && retry.retries == 1 &&
                std::chrono::duration<double>(Clock::now() - retryStart).count() >= 0.95,
                "Retry-After was not respected");
        const auto deadlineStart = Clock::now();
        require(detail::RetryDeadline(deadlineStart, 120, 350) == deadlineStart + std::chrono::seconds(120),
                "long Retry-After was shortened");
        require(detail::RetryDeadline(deadlineStart, std::numeric_limits<long long>::max(), 350) == Clock::time_point::max(),
                "extreme Retry-After overflowed the clock");
        std::atomic<bool> cancelLongRetry{false};
        auto longRetry = std::async(std::launch::async, [&] {
            return cache.Ensure({request("/long-retry/wait")}, nullptr, [&] { return cancelLongRetry.load(); });
        });
        for (int check = 0; check < 100 && requests(base, "/long-retry/wait") == 0; ++check)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cancelLongRetry.store(true);
        require(longRetry.wait_for(std::chrono::seconds(1)) == std::future_status::ready &&
                longRetry.get().cancelled == 1 && requests(base, "/long-retry/wait") == 1,
                "long Retry-After cannot be cancelled promptly");
        {
            DownloadCacheOptions retryOptions = options;
            retryOptions.maxResponseBytes = 256 * 1024;
            retryOptions.maxQueuedRequests = 32;
            DownloadCache retryCache(retryOptions);
            std::vector<DownloadRequest> retryRequests;
            for (int index = 0; index < 32; ++index)
                retryRequests.push_back(request("/large-retry/" + std::to_string(index),
                                                "large-retry-" + std::to_string(index)));
            std::atomic<bool> cancelRetries{false};
            auto waiting = std::async(std::launch::async, [&] {
                return retryCache.Ensure(retryRequests, nullptr, [&] { return cancelRetries.load(); });
            });
            DownloadDiagnostics delayed;
            const auto deadline = Clock::now() + std::chrono::seconds(5);
            do {
                delayed = retryCache.Diagnostics();
                if (delayed.retries == 32 && delayed.active == 0) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (Clock::now() < deadline);
            cancelRetries.store(true);
            const bool promptlyCancelled = waiting.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
            if (!promptlyCancelled) retryCache.Stop();
            const auto cancelledRetries = waiting.get();
            std::cout << "Delayed retry buffers after 32 x 128 KiB failures: "
                      << delayed.bufferedBytes << " bytes\n";
            require(delayed.retries == 32 && delayed.queued == 32 && delayed.active == 0,
                    "large-response retry fixture did not enter backoff");
            require(delayed.bufferedBytes == 0,
                    "delayed retries retain failed response buffers beyond active transfer limits");
            require(promptlyCancelled && cancelledRetries.cancelled == 32,
                    "large-response retry waits cannot be cancelled promptly");
        }
        auto oversized = cache.Ensure({request("/oversize/large")});
        require(oversized.failed == 1 && !fs::exists(root / "large"), "oversized body published");
        auto owner = std::async(std::launch::async, [&] { return cache.Ensure({request("/slow/shared")}); });
        std::atomic<bool> cancel{false};
        auto subscriber = std::async(std::launch::async, [&] {
            return cache.Ensure({request("/slow/shared")}, nullptr, [&] { return cancel.load(); });
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); cancel.store(true);
        require(subscriber.get().cancelled == 1, "subscriber cancellation failed");
        require(owner.get().downloaded == 1 && requests(base, "/slow/shared") == 1,
                "cancelled subscriber aborted another caller's transfer");
        // A separate exec'ed process shares flock with this service; it must not
        // redownload a file published by the first writer.
        const auto rootText = root.string();
        char *childArgv[] = {argv[0], argv[1], const_cast<char *>(rootText.c_str()),
                             const_cast<char *>("--child"), nullptr};
        pid_t child = 0;
        require(posix_spawn(&child, argv[0], nullptr, nullptr, childArgv, environ) == 0, "spawn failed");
        auto process = cache.Ensure({request("/slow/process", "process")});
        int status = 0; waitpid(child, &status, 0);
        require(WIFEXITED(status) && WEXITSTATUS(status) == 0 && process.failed == 0 &&
                requests(base, "/slow/process") == 1, "cross-process duplicate transfer");
        const auto locked = root / "locked";
        const int lock = open((locked.string() + ".lock").c_str(), O_CREAT | O_RDWR, 0600);
        require(lock >= 0 && flock(lock, LOCK_EX) == 0, "fixture lock failed");
        cancel.store(false);
        auto blocked = std::async(std::launch::async, [&] {
            return cache.Ensure({request("/tile/locked")}, nullptr, [&] { return cancel.load(); });
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(60)); cancel.store(true);
        require(blocked.wait_for(std::chrono::seconds(1)) == std::future_status::ready &&
                blocked.get().cancelled == 1, "flock wait cannot be cancelled");
        flock(lock, LOCK_UN); close(lock);
        auto stalled = std::async(std::launch::async, [&] { return cache.Ensure({request("/delay/stalled")}); });
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        const auto stopStart = Clock::now(); cache.Stop();
        require(stalled.get().cancelled == 1 &&
                std::chrono::duration<double>(Clock::now() - stopStart).count() < 1.0,
                "shutdown waited for HTTP timeout");
        require(cache.Ensure({request("/tile/afterstop")}).cancelled == 1, "stopped cache accepted work");
        for (const auto &entry : fs::directory_iterator(root))
            require(entry.path().string().find(".part.") == std::string::npos, "temporary file leaked");
        std::cout << "Download cache: connection reuse, corrupt recovery, retries, limits, cancellation, flock passed\n";
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
