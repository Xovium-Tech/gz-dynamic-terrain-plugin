#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace dynamic_terrain
{
// Simulator-independent transport: this header and its library need no image,
// simulator, or rendering SDK. Validators run on the download worker thread.
using DownloadValidator = std::function<bool(const std::vector<unsigned char> &)>;
using DownloadCancelCheck = std::function<bool()>;

// Stable across processes/simulators; custom dataset identity excludes runtime
// token substitution. Empty URL retains the established built-in provider path.
std::string TerrainCacheNamespace(const std::string &provider,
                                  const std::string &urlTemplate,
                                  const std::string &encoding);

struct DownloadRequest
{
    std::string label;
    std::string url;
    std::filesystem::path path;
    DownloadValidator validate;
};

struct DownloadCacheOptions
{
    int concurrency{4};
    int perHost{1};
    int retries{3};
    unsigned int timeoutMs{5000};
    std::size_t maxResponseBytes{32u * 1024u * 1024u};
    std::size_t maxQueuedRequests{4096};
    std::string userAgent{"dynamic-terrain/0.2.1"};
};

struct DownloadStats
{
    std::size_t requested{0};
    std::size_t cached{0};
    std::size_t downloaded{0};
    std::size_t failed{0};
    std::size_t cancelled{0};
    std::size_t bytes{0};
    std::size_t connections{0};
    std::size_t retries{0};
    double elapsedSec{0.0};
};

// Service-wide counters count each shared transfer once, independent of the
// number of callers waiting for it. Queued includes delayed retries/locks.
struct DownloadDiagnostics
{
    std::size_t queued{0}, active{0}, peakQueued{0}, peakActive{0};
    std::size_t bufferedBytes{0};
    std::size_t cacheHits{0}, downloaded{0}, failed{0}, cancelled{0};
    std::size_t bytes{0}, connections{0}, retries{0};
};

class DownloadCache
{
public:
    explicit DownloadCache(DownloadCacheOptions options = {});
    ~DownloadCache();
    DownloadCache(const DownloadCache &) = delete;
    DownloadCache &operator=(const DownloadCache &) = delete;

    // Thread-safe, blocking only the caller. Concurrent calls share transfers.
    // Larger priorities start first; running transfers are never preempted while
    // another caller still requires them. Cancellation is checked on the caller
    // thread, and only withdraws this caller's interest in shared requests.
    DownloadStats Ensure(const std::vector<DownloadRequest> &requests,
                         std::vector<std::filesystem::path> *failedPaths = nullptr,
                         DownloadCancelCheck cancel = {}, int priority = 0) const;
    // Interrupts transfers and waits for worker shutdown. Idempotent.
    void Stop() const;
    DownloadDiagnostics Diagnostics() const;
    // Drop only the in-memory validation stamp after an adapter observes a
    // decode failure. The next Ensure validates/repairs under the normal lock.
    void InvalidateValidation(const std::filesystem::path &path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
