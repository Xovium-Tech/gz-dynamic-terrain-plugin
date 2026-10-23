#include "dynamic_terrain/core/DownloadCache.hh"
#include "DownloadRetry.hh"

#include <curl/curl.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cctype>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace dynamic_terrain
{
namespace
{
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
enum class Outcome { Pending, Cached, Downloaded, Failed, Cancelled };
struct Result
{
    Outcome outcome{Outcome::Pending};
    std::size_t bytes{0}, connections{0}, retries{0};
};
struct Batch
{
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<bool> cancelled{false};
    std::vector<Result> results;
    std::size_t finished{0};
};
struct Subscriber { std::shared_ptr<Batch> batch; std::size_t index; };
struct Task
{
    DownloadRequest request;
    fs::path localSource;
    std::string key, host;
    int priority{0}, attempt{0}, lock{-1};
    std::size_t sequence{0}, maxBytes{0}, connections{0};
    std::vector<Subscriber> subscribers;
    std::vector<unsigned char> bytes;
    std::atomic<std::size_t> bufferCapacity{0};
    std::atomic<std::size_t> receivedBytes{0};
    CURL *easy{nullptr};
    Clock::time_point ready{Clock::now()};
    ~Task() { if (lock >= 0) { flock(lock, LOCK_UN); close(lock); } }
};

std::string hostFor(const std::string &url)
{
    CURLU *parsed = curl_url();
    if (!parsed) return {};
    char *host = nullptr, *scheme = nullptr, *port = nullptr;
    std::string result;
    if (curl_url_set(parsed, CURLUPART_URL, url.c_str(), 0) == CURLUE_OK &&
        curl_url_get(parsed, CURLUPART_HOST, &host, 0) == CURLUE_OK &&
        curl_url_get(parsed, CURLUPART_SCHEME, &scheme, 0) == CURLUE_OK &&
        (std::string(scheme) == "https" || std::string(scheme) == "http"))
    {
        curl_url_get(parsed, CURLUPART_PORT, &port, CURLU_DEFAULT_PORT);
        result = std::string(host) + ":" + (port ? port : "");
    }
    curl_free(host); curl_free(scheme); curl_free(port); curl_url_cleanup(parsed);
    return result;
}

fs::path localSourceFor(const std::string &url)
{
    if (!url.empty() && url.front() == '/') return fs::path(url);
    if (url.rfind("file://", 0) != 0) return {};
    const auto slash = url.find('/', 7);
    if (slash == std::string::npos) return {};
    const auto host = url.substr(7, slash - 7);
    if (!host.empty() && host != "localhost") return {};
    const auto encoded = url.substr(slash);
    int length = 0;
    char *decoded = curl_easy_unescape(nullptr, encoded.c_str(), static_cast<int>(encoded.size()), &length);
    if (!decoded) return {};
    std::string path(decoded, static_cast<std::size_t>(length)); curl_free(decoded);
    if (path.find('\0') != std::string::npos) return {};
    return fs::path(path);
}

bool readLocal(const fs::path &path, std::size_t limit, std::vector<unsigned char> &bytes)
{
    std::error_code error;
    if (!fs::is_regular_file(path, error)) return false;
    const auto size = fs::file_size(path, error);
    if (error || size == 0 || size > limit) return false;
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    bytes.resize(static_cast<std::size_t>(size));
    file.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(size));
    return static_cast<bool>(file);
}

size_t receive(void *data, size_t size, size_t count, void *opaque) noexcept
{
    auto &task = *static_cast<Task *>(opaque);
    if (size && count > task.maxBytes / size) return 0;
    const auto amount = size * count;
    if (amount > task.maxBytes - task.bytes.size()) return 0;
    try
    {
        const auto *begin = static_cast<unsigned char *>(data);
        task.bytes.insert(task.bytes.end(), begin, begin + amount);
        task.bufferCapacity.store(task.bytes.capacity(), std::memory_order_relaxed);
        task.receivedBytes.fetch_add(amount, std::memory_order_relaxed);
        return amount;
    }
    catch (...) { return 0; }
}

bool validBytes(const DownloadRequest &request, const std::vector<unsigned char> &bytes)
{
    try { return !bytes.empty() && (!request.validate || request.validate(bytes)); }
    catch (...) { return false; }
}

std::string certificateBundle()
{
    for (const char *name : {"CURL_CA_BUNDLE", "SSL_CERT_FILE"})
        if (const char *value = std::getenv(name); value && *value) return value;
    for (const char *path : {"/etc/ssl/certs/ca-certificates.crt",
                             "/etc/pki/tls/certs/ca-bundle.crt", "/etc/ssl/ca-bundle.pem"})
    {
        std::error_code error;
        if (fs::is_regular_file(path, error)) return path;
    }
    return {};
}

bool publish(const fs::path &path, const std::vector<unsigned char> &bytes)
{
    // mkstemp guarantees uniqueness across processes. Keeping the temporary file
    // beside the target makes rename atomic for every reader on this filesystem.
    std::string name = path.string() + ".part.XXXXXX";
    std::vector<char> temporary(name.begin(), name.end()); temporary.push_back('\0');
    const int fd = mkstemp(temporary.data());
    if (fd < 0) return false;
    std::size_t offset = 0;
    bool ok = true;
    while (offset < bytes.size())
    {
        const auto written = write(fd, bytes.data() + offset, bytes.size() - offset);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) { ok = false; break; }
        offset += static_cast<std::size_t>(written);
    }
    if (close(fd) != 0) ok = false;
    std::error_code error;
    if (ok) { fs::rename(temporary.data(), path, error); ok = !error; }
    if (!ok) fs::remove(temporary.data(), error);
    return ok;
}

bool transient(CURLcode code, long status)
{
    if (status == 408 || status == 425 || status == 429 || status >= 500) return true;
    if (status >= 400) return false;
    return code == CURLE_COULDNT_RESOLVE_HOST || code == CURLE_COULDNT_RESOLVE_PROXY ||
        code == CURLE_COULDNT_CONNECT || code == CURLE_OPERATION_TIMEDOUT ||
        code == CURLE_RECV_ERROR || code == CURLE_SEND_ERROR ||
        code == CURLE_PARTIAL_FILE || code == CURLE_GOT_NOTHING ||
        code == CURLE_HTTP2 || code == CURLE_HTTP2_STREAM;
}
}

struct DownloadCache::Impl
{
    DownloadCacheOptions options;
    mutable std::mutex mutex, stopMutex;
    mutable std::condition_variable cv;
    std::atomic<bool> stopped{false};
    std::unordered_map<std::string, std::shared_ptr<Task>> tasks;
    std::unordered_map<CURL *, std::shared_ptr<Task>> active;
    std::unordered_map<std::string, int> hosts;
    std::unordered_map<std::string, bool> multiplexed;
    struct Stamp { fs::file_time_type modified; std::uintmax_t bytes; };
    std::unordered_map<std::string, Stamp> validated;
    std::size_t sequence{0};
    DownloadDiagnostics totals;
    std::thread worker;

    explicit Impl(DownloadCacheOptions cfg) : options(std::move(cfg))
    {
        options.concurrency = std::clamp(options.concurrency, 1, 64);
        options.perHost = std::clamp(options.perHost, 1, options.concurrency);
        options.retries = std::clamp(options.retries, 0, 10);
        options.timeoutMs = std::max(1u, options.timeoutMs);
        options.maxResponseBytes = std::max<std::size_t>(1, options.maxResponseBytes);
        options.maxQueuedRequests = std::max<std::size_t>(options.concurrency, options.maxQueuedRequests);
        static std::once_flag initialized;
        static CURLcode initResult = CURLE_FAILED_INIT;
        std::call_once(initialized, [] { initResult = curl_global_init(CURL_GLOBAL_DEFAULT); });
        if (initResult != CURLE_OK) throw std::runtime_error("libcurl initialization failed");
        worker = std::thread([this] { run(); });
    }
    ~Impl() { stop(); }
    void stop()
    {
        std::lock_guard<std::mutex> joining(stopMutex);
        stopped.store(true); cv.notify_all();
        if (worker.joinable()) worker.join();
    }
    bool wanted(const Task &task) const
    {
        return std::any_of(task.subscribers.begin(), task.subscribers.end(),
            [](const Subscriber &s) { return !s.batch->cancelled.load(); });
    }
    // Called with the service mutex, never with a batch mutex held by caller.
    void finish(const std::shared_ptr<Task> &task, Outcome outcome)
    {
        switch (outcome)
        {
            case Outcome::Cached: ++totals.cacheHits; break;
            case Outcome::Downloaded: ++totals.downloaded; break;
            case Outcome::Failed: ++totals.failed; break;
            case Outcome::Cancelled: ++totals.cancelled; break;
            default: break;
        }
        totals.bytes += task->receivedBytes.load(std::memory_order_relaxed);
        totals.connections += task->connections;
        totals.retries += static_cast<std::size_t>(task->attempt);
        for (const auto &s : task->subscribers)
        {
            std::lock_guard<std::mutex> lock(s.batch->mutex);
            auto &result = s.batch->results[s.index];
            if (result.outcome != Outcome::Pending) continue;
            result = {s.batch->cancelled.load() ? Outcome::Cancelled : outcome,
                      task->bytes.size(), task->connections,
                      static_cast<std::size_t>(task->attempt)};
            ++s.batch->finished; s.batch->cv.notify_all();
        }
        // A scheduling pass still owns completed local tasks in its pending
        // vector. Release their payloads and tile locks now, rather than
        // retaining one response and descriptor per queued local source.
        std::vector<unsigned char>().swap(task->bytes);
        task->bufferCapacity.store(0, std::memory_order_relaxed);
        if (task->lock >= 0)
        {
            flock(task->lock, LOCK_UN); close(task->lock); task->lock = -1;
        }
        tasks.erase(task->key); cv.notify_all();
    }
    DownloadDiagnostics diagnostics() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        auto result = totals;
        result.active = active.size();
        result.queued = tasks.size() - active.size();
        for (const auto &entry : tasks)
        {
            result.bufferedBytes += entry.second->bufferCapacity.load(std::memory_order_relaxed);
            result.bytes += entry.second->receivedBytes.load(std::memory_order_relaxed);
            result.connections += entry.second->connections;
            result.retries += static_cast<std::size_t>(entry.second->attempt);
        }
        return result;
    }
    bool validFile(Task &task)
    {
        std::error_code ec;
        if (!fs::is_regular_file(task.request.path, ec)) return false;
        const auto size = fs::file_size(task.request.path, ec);
        if (ec || size == 0 || size > options.maxResponseBytes) return false;
        const auto modified = fs::last_write_time(task.request.path, ec);
        if (ec) return false;
        const auto stamp = validated.find(task.key);
        if (stamp != validated.end() && stamp->second.modified == modified &&
            stamp->second.bytes == size) return true;
        std::ifstream input(task.request.path, std::ios::binary);
        if (!input) return false;
        std::vector<unsigned char> bytes(static_cast<std::size_t>(size));
        input.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(size));
        if (!input || !validBytes(task.request, bytes)) return false;
        // Bounded metadata cache: eviction affects work only, never tile content.
        if (validated.size() >= options.maxQueuedRequests * 2) validated.clear();
        validated[task.key] = {modified, size};
        return true;
    }
    void removeEasy(CURLM *multi, Task &task)
    {
        if (!task.easy) return;
        curl_multi_remove_handle(multi, task.easy);
        active.erase(task.easy);
        curl_easy_cleanup(task.easy); task.easy = nullptr;
        --hosts[task.host];
    }
    bool start(CURLM *multi, const std::shared_ptr<Task> &task)
    {
        // A valid cache hit needs no lock; writers only publish by atomic rename.
        if (validFile(*task)) { finish(task, Outcome::Cached); return true; }
        if (task->host.empty() && task->localSource.empty()) { finish(task, Outcome::Failed); return true; }
        if (task->lock < 0)
        {
            std::error_code ec; fs::create_directories(task->request.path.parent_path(), ec);
            if (ec) { finish(task, Outcome::Failed); return true; }
            const std::string lockName = task->request.path.string() + ".lock";
            const int fd = open(lockName.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
            if (fd < 0) { finish(task, Outcome::Failed); return true; }
            if (flock(fd, LOCK_EX | LOCK_NB) != 0)
            {
                const int reason = errno; close(fd);
                if (reason != EWOULDBLOCK && reason != EAGAIN) { finish(task, Outcome::Failed); return true; }
                task->ready = Clock::now() + std::chrono::milliseconds(25);
                return false;
            }
            task->lock = fd;
            if (validFile(*task)) { finish(task, Outcome::Cached); return true; }
        }
        if (!task->localSource.empty())
        {
            const bool loaded = readLocal(task->localSource, options.maxResponseBytes, task->bytes);
            task->bufferCapacity.store(task->bytes.capacity(), std::memory_order_relaxed);
            const bool ready = loaded &&
                validBytes(task->request, task->bytes) && publish(task->request.path, task->bytes);
            validated.erase(task->key);
            finish(task, ready ? Outcome::Downloaded : Outcome::Failed);
            return true;
        }
        task->easy = curl_easy_init();
        if (!task->easy) { finish(task, Outcome::Failed); return true; }
        task->bytes.clear(); task->maxBytes = options.maxResponseBytes;
        curl_easy_setopt(task->easy, CURLOPT_URL, task->request.url.c_str());
        const auto ca = certificateBundle();
        if (!ca.empty()) curl_easy_setopt(task->easy, CURLOPT_CAINFO, ca.c_str());
        curl_easy_setopt(task->easy, CURLOPT_SSL_VERIFYPEER, 1L);
        curl_easy_setopt(task->easy, CURLOPT_SSL_VERIFYHOST, 2L);
        curl_easy_setopt(task->easy, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(task->easy, CURLOPT_MAXREDIRS, 5L);
#if LIBCURL_VERSION_NUM >= 0x075500
        curl_easy_setopt(task->easy, CURLOPT_PROTOCOLS_STR, "http,https");
        curl_easy_setopt(task->easy, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
#else
        curl_easy_setopt(task->easy, CURLOPT_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
        curl_easy_setopt(task->easy, CURLOPT_REDIR_PROTOCOLS, CURLPROTO_HTTP | CURLPROTO_HTTPS);
#endif
        curl_easy_setopt(task->easy, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(task->easy, CURLOPT_TCP_KEEPALIVE, 1L);
        curl_easy_setopt(task->easy, CURLOPT_TIMEOUT_MS, static_cast<long>(options.timeoutMs));
        curl_easy_setopt(task->easy, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(std::min(options.timeoutMs, 3000u)));
        curl_easy_setopt(task->easy, CURLOPT_USERAGENT, options.userAgent.c_str());
        curl_easy_setopt(task->easy, CURLOPT_PIPEWAIT, 1L);
        curl_easy_setopt(task->easy, CURLOPT_WRITEFUNCTION, receive);
        curl_easy_setopt(task->easy, CURLOPT_WRITEDATA, task.get());
        if (curl_multi_add_handle(multi, task->easy) != CURLM_OK)
        {
            curl_easy_cleanup(task->easy); task->easy = nullptr;
            finish(task, Outcome::Failed); return true;
        }
        active.emplace(task->easy, task); ++hosts[task->host];
        totals.peakActive = std::max(totals.peakActive, active.size());
        return true;
    }
    void run() noexcept
    {
        CURLM *multi = curl_multi_init();
        try
        {
            if (!multi) throw std::runtime_error("libcurl multi initialization failed");
            curl_multi_setopt(multi, CURLMOPT_MAX_TOTAL_CONNECTIONS, static_cast<long>(options.concurrency));
            curl_multi_setopt(multi, CURLMOPT_MAX_HOST_CONNECTIONS, static_cast<long>(options.perHost));
            curl_multi_setopt(multi, CURLMOPT_MAXCONNECTS, static_cast<long>(options.concurrency * 2));
            while (!stopped.load())
            {
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    std::vector<std::shared_ptr<Task>> pending, unwanted;
                    for (const auto &entry : tasks)
                    {
                        const auto &task = entry.second;
                        if (!wanted(*task)) unwanted.push_back(task);
                        else if (!task->easy && task->ready <= Clock::now()) pending.push_back(task);
                    }
                    for (const auto &task : unwanted) { removeEasy(multi, *task); finish(task, Outcome::Cancelled); }
                    std::sort(pending.begin(), pending.end(), [](const auto &a, const auto &b)
                    {
                        return a->priority != b->priority ? a->priority > b->priority : a->sequence < b->sequence;
                    });
                    for (const auto &task : pending)
                    {
                        if (stopped.load()) break;
                        if (!wanted(*task)) { finish(task, Outcome::Cancelled); continue; }
                        if (active.size() >= static_cast<std::size_t>(options.concurrency)) break;
                        const int window = multiplexed[task->host] ? options.concurrency : options.perHost;
                        if (hosts[task->host] < window) start(multi, task);
                    }
                    if (stopped.load()) break;
                    if (active.empty())
                    {
                        cv.wait_for(lock, std::chrono::milliseconds(25));
                        continue;
                    }
                }
                int running = 0;
                if (curl_multi_perform(multi, &running) != CURLM_OK)
                    throw std::runtime_error("libcurl multi transfer failure");
                int remaining = 0;
                bool completed = false;
                while (CURLMsg *message = curl_multi_info_read(multi, &remaining))
                {
                    if (message->msg != CURLMSG_DONE) continue;
                    std::lock_guard<std::mutex> lock(mutex);
                    auto found = active.find(message->easy_handle);
                    if (found == active.end()) continue;
                    completed = true;
                    auto task = found->second;
                    long status = 0, connects = 0, httpVersion = 0;
                    curl_off_t retryAfter = 0;
                    curl_easy_getinfo(task->easy, CURLINFO_RESPONSE_CODE, &status);
                    curl_easy_getinfo(task->easy, CURLINFO_NUM_CONNECTS, &connects);
                    curl_easy_getinfo(task->easy, CURLINFO_HTTP_VERSION, &httpVersion);
                    if (httpVersion != 0)
                        multiplexed[task->host] = httpVersion >= CURL_HTTP_VERSION_2_0;
#if LIBCURL_VERSION_NUM >= 0x074200
                    curl_easy_getinfo(task->easy, CURLINFO_RETRY_AFTER, &retryAfter);
#endif
                    task->connections += static_cast<std::size_t>(std::max(0L, connects));
                    const CURLcode code = message->data.result;
                    removeEasy(multi, *task);
                    const bool valid = code == CURLE_OK && status >= 200 && status < 300 &&
                        validBytes(task->request, task->bytes);
                    if (valid && publish(task->request.path, task->bytes))
                    {
                        validated.erase(task->key);
                        finish(task, Outcome::Downloaded);
                    }
                    else if (task->attempt < options.retries &&
                             (transient(code, status) || (code == CURLE_OK && status >= 200 && status < 300 && !valid)))
                    {
                        const auto backoff = std::min(2000, 250 * (1 << task->attempt));
                        // Small deterministic jitter avoids identical cross-process retries.
                        const auto jitter = static_cast<int>((task->sequence * 37u + static_cast<unsigned>(getpid())) % 101u);
                        ++task->attempt;
                        task->ready = detail::RetryDeadline(Clock::now(), retryAfter, backoff + jitter);
                        // Failed payloads are never reused by the next attempt.
                        // Delayed retries do not occupy a transfer slot, so
                        // keeping their capacity would multiply the response
                        // memory bound by the entire queue during an outage.
                        std::vector<unsigned char>().swap(task->bytes);
                        task->bufferCapacity.store(0, std::memory_order_relaxed);
                    }
                    else finish(task, Outcome::Failed);
                }
                // Refill freed slots before polling. Polling an empty multi
                // handle inserts the full timeout between otherwise immediate
                // HTTP/1 requests, limiting a warm local connection to 40/s.
                if (completed) continue;
                int descriptors = 0;
#if LIBCURL_VERSION_NUM >= 0x074200
                curl_multi_poll(multi, nullptr, 0, 25, &descriptors);
#else
                curl_multi_wait(multi, nullptr, 0, 25, &descriptors);
#endif
            }
        }
        catch (...) { stopped.store(true); }
        std::lock_guard<std::mutex> lock(mutex);
        while (!tasks.empty())
        {
            const auto task = tasks.begin()->second;
            if (multi) removeEasy(multi, *task);
            finish(task, Outcome::Cancelled);
        }
        if (multi) curl_multi_cleanup(multi);
        cv.notify_all();
    }

    DownloadStats ensure(const std::vector<DownloadRequest> &requests,
                         std::vector<fs::path> *failedPaths,
                         const DownloadCancelCheck &cancel, int priority)
    {
        const auto began = Clock::now();
        DownloadStats stats; stats.requested = requests.size();
        auto batch = std::make_shared<Batch>();
        std::vector<DownloadRequest> unique;
        std::unordered_set<std::string> seen;
        for (const auto &request : requests)
        {
            std::error_code ec;
            auto normalized = fs::absolute(request.path, ec).lexically_normal();
            const std::string key = ec ? request.path.string() : normalized.string();
            if (seen.insert(key).second) { unique.push_back(request); unique.back().path = key; }
        }
        batch->results.resize(unique.size());
        for (std::size_t i = 0; i < unique.size(); ++i)
        {
            auto &request = unique[i];
            const std::string key = request.path.string();
            std::unique_lock<std::mutex> lock(mutex);
            while (!stopped.load() && tasks.size() >= options.maxQueuedRequests && tasks.find(key) == tasks.end())
            {
                lock.unlock();
                if (cancel && cancel()) batch->cancelled.store(true);
                lock.lock();
                if (batch->cancelled.load()) break;
                cv.wait_for(lock, std::chrono::milliseconds(25));
            }
            if (stopped.load() || batch->cancelled.load()) break;
            auto found = tasks.find(key);
            if (found != tasks.end())
            {
                if (found->second->request.url != request.url)
                {
                    std::lock_guard<std::mutex> batchLock(batch->mutex);
                    batch->results[i].outcome = Outcome::Failed; ++batch->finished;
                    continue;
                }
                found->second->subscribers.push_back({batch, i});
                found->second->priority = std::max(found->second->priority, priority);
            }
            else
            {
                auto task = std::make_shared<Task>(); task->request = request;
                task->key = key; task->host = hostFor(request.url);
                task->localSource = localSourceFor(request.url);
                task->priority = priority; task->sequence = ++sequence;
                task->subscribers.push_back({batch, i}); tasks.emplace(key, task);
                totals.peakQueued = std::max(totals.peakQueued, tasks.size() - active.size());
            }
            cv.notify_all();
        }
        for (;;)
        {
            if (stopped.load() || (cancel && cancel())) { batch->cancelled.store(true); cv.notify_all(); break; }
            std::unique_lock<std::mutex> lock(batch->mutex);
            if (batch->finished == batch->results.size()) break;
            batch->cv.wait_for(lock, std::chrono::milliseconds(25));
        }
        std::lock_guard<std::mutex> lock(batch->mutex);
        for (std::size_t i = 0; i < batch->results.size(); ++i)
        {
            const auto &result = batch->results[i];
            switch (result.outcome)
            {
                case Outcome::Cached: ++stats.cached; break;
                case Outcome::Downloaded: ++stats.downloaded; break;
                case Outcome::Failed: ++stats.failed; break;
                default: ++stats.cancelled; break;
            }
            if (result.outcome != Outcome::Cached && result.outcome != Outcome::Downloaded && failedPaths)
                failedPaths->push_back(unique[i].path);
            stats.bytes += result.bytes; stats.connections += result.connections; stats.retries += result.retries;
        }
        stats.elapsedSec = std::chrono::duration<double>(Clock::now() - began).count();
        return stats;
    }
};

DownloadCache::DownloadCache(DownloadCacheOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}
DownloadCache::~DownloadCache() = default;
DownloadStats DownloadCache::Ensure(const std::vector<DownloadRequest> &requests,
                                   std::vector<std::filesystem::path> *failedPaths,
                                   DownloadCancelCheck cancel, int priority) const
{ return impl_->ensure(requests, failedPaths, cancel, priority); }
void DownloadCache::Stop() const { impl_->stop(); }
DownloadDiagnostics DownloadCache::Diagnostics() const { return impl_->diagnostics(); }
void DownloadCache::InvalidateValidation(const std::filesystem::path &path) const
{
    std::error_code error;
    const auto normalized = std::filesystem::absolute(path, error).lexically_normal();
    const auto key = error ? path.string() : normalized.string();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->validated.erase(key);
}

std::string TerrainCacheNamespace(const std::string &provider,
                                  const std::string &urlTemplate,
                                  const std::string &encoding)
{
    std::string name = provider;
    for (auto &c : name)
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_' && c != '-') c = '_';
    if (urlTemplate.empty()) return name;
    // FNV-1a is specified here rather than std::hash, whose output is not a
    // portable persistent identifier. No URL or credentials appear in names.
    std::uint64_t hash = 14695981039346656037ull;
    for (unsigned char c : provider + "\n" + urlTemplate + "\n" + encoding)
    { hash ^= c; hash *= 1099511628211ull; }
    std::ostringstream suffix; suffix << std::hex << std::setfill('0') << std::setw(16) << hash;
    return name + "_" + suffix.str();
}
}
