#include "dynamic_terrain/core/DownloadCache.hh"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace
{
namespace fs = std::filesystem;
using namespace dynamic_terrain;
using namespace std::chrono_literals;
void require(bool value, const char *message)
{ if (!value) throw std::runtime_error(message); }
std::size_t rss()
{
    std::ifstream file("/proc/self/statm"); std::size_t total = 0, resident = 0;
    file >> total >> resident;
    return resident * static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
}
std::size_t descriptors()
{
    return static_cast<std::size_t>(std::distance(fs::directory_iterator("/proc/self/fd"),
                                                fs::directory_iterator{}));
}
struct Locks
{
    std::vector<int> values;
    void clear() { for (int fd : values) { flock(fd, LOCK_UN); close(fd); } values.clear(); }
    ~Locks() { clear(); }
};
}

int main(int argc, char **argv)
{
    try
    {
        require(argc == 3, "usage: DownloadLocal_TEST CACHE memory|cancel|stop");
        const fs::path root = argv[1]; const std::string mode = argv[2];
        require(mode == "memory" || mode == "cancel" || mode == "stop", "unknown mode");
        fs::remove_all(root); fs::create_directories(root);
        constexpr std::size_t count = 64, body = 1024 * 1024;
        {
            const std::vector<char> bytes(body, 'X');
            std::ofstream file(root / "source.bin", std::ios::binary);
            file.write(bytes.data(), bytes.size());
        }
        Locks locks;
        std::vector<DownloadRequest> requests;
        std::atomic<bool> cancel{false};
        std::atomic<std::size_t> validations{0};
        std::promise<void> entered, release;
        auto enteredFuture = entered.get_future();
        auto releaseFuture = release.get_future().share();
        std::size_t firstRss = 0, peakRss = 0, firstFds = 0, peakFds = 0;
        for (std::size_t index = 0; index < count; ++index)
        {
            const auto destination = root / (std::to_string(index) + ".bin");
            const int fd = open((destination.string() + ".lock").c_str(), O_CREAT | O_RDWR, 0600);
            require(fd >= 0, "fixture lock open failed"); locks.values.push_back(fd);
            require(flock(fd, LOCK_EX) == 0, "fixture lock failed");
            requests.push_back({"local", "file://" + (root / "source.bin").string(), destination,
                [&](const auto &bytes)
                {
                    const auto index = validations.fetch_add(1);
                    const auto memory = rss(), fds = descriptors();
                    if (index == 0) { firstRss = memory; firstFds = fds; }
                    peakRss = std::max(peakRss, memory); peakFds = std::max(peakFds, fds);
                    if (mode != "memory" && index == 0)
                    {
                        entered.set_value();
                        if (releaseFuture.wait_for(5s) != std::future_status::ready) return false;
                    }
                    return bytes.size() == body && bytes.front() == 'X';
                }});
        }
        DownloadCacheOptions options;
        options.concurrency = 2; options.perHost = 1; options.retries = 0;
        options.maxQueuedRequests = count + 1;
        DownloadCache cache(options);
        auto caller = std::async(std::launch::async, [&]
        { return cache.Ensure(requests, nullptr, [&] { return cancel.load(); }); });
        // Pre-held locks let the caller enqueue the entire batch before the
        // worker can process local files synchronously in one scheduling pass.
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (cache.Diagnostics().queued != count && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(1ms);
        const bool enqueued = cache.Diagnostics().queued == count;
        locks.clear();
        std::future<void> stopping;
        bool cancellationPrompt = true;
        if (mode != "memory")
        {
            const bool firstEntered = enteredFuture.wait_for(5s) == std::future_status::ready;
            if (mode == "stop") stopping = std::async(std::launch::async, [&] { cache.Stop(); });
            else cancel.store(true);
            cancellationPrompt = firstEntered && caller.wait_for(1s) == std::future_status::ready;
            release.set_value();
        }
        const auto stats = caller.get();
        if (stopping.valid()) stopping.get();
        // In cancellation mode Diagnostics waits for the current scheduling
        // pass to finish; Stop must not mask a missing subscriber check.
        const auto diagnostics = cache.Diagnostics();
        cache.Stop();
        std::size_t published = 0;
        for (std::size_t index = 0; index < count; ++index)
            published += fs::exists(root / (std::to_string(index) + ".bin"));
        std::cout << "{\"mode\":\"" << mode << "\",\"validations\":" << validations.load()
                  << ",\"published\":" << published << ",\"first_rss_bytes\":" << firstRss
                  << ",\"peak_rss_bytes\":" << peakRss << ",\"first_fds\":" << firstFds
                  << ",\"peak_fds\":" << peakFds << ",\"reported_bytes\":" << stats.bytes << "}\n";
        require(enqueued, "local batch was not fully queued");
        if (mode == "memory")
        {
            require(stats.downloaded == count && stats.bytes == count * body && published == count,
                    "local file contents or completed byte accounting changed");
            require(peakRss <= firstRss + 8 * body,
                    "completed local files retain payloads for the whole scheduling pass");
            require(peakFds <= firstFds + 1,
                    "completed local files retain tile lock descriptors for the whole scheduling pass");
        }
        else
        {
            require(cancellationPrompt && stats.cancelled == count,
                    "local batch cancellation or shutdown was not prompt");
            require(validations.load() == 1 && published == 1,
                    "queued local files started after cancellation or shutdown");
        }
        require(diagnostics.bufferedBytes == 0, "terminal local payloads remain allocated");
        fs::remove_all(root);
        return 0;
    }
    catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
