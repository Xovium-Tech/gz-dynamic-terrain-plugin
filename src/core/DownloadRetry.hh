#pragma once

#include <algorithm>
#include <chrono>

namespace dynamic_terrain::detail
{
// Saturation prevents hostile or very distant Retry-After values overflowing
// the clock while preserving the provider's minimum delay. The scheduler can
// still cancel a pending deadline immediately.
inline std::chrono::steady_clock::time_point RetryDeadline(
    std::chrono::steady_clock::time_point now,
    long long retryAfterSeconds, int backoffMilliseconds)
{
    using Clock = std::chrono::steady_clock;
    const auto remaining = Clock::time_point::max() - now;
    const auto maximumSeconds = std::chrono::duration_cast<std::chrono::seconds>(remaining).count();
    if (retryAfterSeconds > maximumSeconds) return Clock::time_point::max();
    const auto delay = std::max(
        std::chrono::duration_cast<Clock::duration>(std::chrono::seconds(std::max(0LL, retryAfterSeconds))),
        std::chrono::duration_cast<Clock::duration>(std::chrono::milliseconds(backoffMilliseconds)));
    return delay >= remaining ? Clock::time_point::max() : now + delay;
}
}
