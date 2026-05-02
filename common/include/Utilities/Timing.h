#ifndef UTILITIES_TIMING_H
#define UTILITIES_TIMING_H

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>

namespace profiling {

inline bool& profilingEnabledFlag() {
    static bool enabled = false;
    return enabled;
}

inline bool enabled() {
    return profilingEnabledFlag();
}

inline void setEnabled(const bool enabled) {
    profilingEnabledFlag() = enabled;
}

inline bool parseBooleanEnvValue(const char* value) {
    if (value == nullptr) {
        return false;
    }

    std::string normalized(value);
    normalized.erase(
        std::remove_if(normalized.begin(),
                       normalized.end(),
                       [](unsigned char ch) { return std::isspace(ch) != 0; }),
        normalized.end());
    std::transform(normalized.begin(),
                   normalized.end(),
                   normalized.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

    if (normalized.empty()) {
        return false;
    }
    if (normalized == "1" || normalized == "true" || normalized == "yes" ||
        normalized == "on") {
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "no" ||
        normalized == "off") {
        return false;
    }

    return false;
}

inline void configureFromEnvironment(const char* envName = "CONVEXMPC_PROFILE_TIMING") {
    setEnabled(parseBooleanEnvValue(std::getenv(envName)));
}

struct TimingStats {
    using clock = std::chrono::steady_clock;

    std::uint64_t samples{0};
    std::chrono::nanoseconds total{0};
    std::chrono::nanoseconds max{0};

    void add(const std::chrono::nanoseconds sample) {
        ++samples;
        total += sample;
        if (sample > max) {
            max = sample;
        }
    }

    bool empty() const {
        return samples == 0;
    }

    double averageMilliseconds() const {
        if (samples == 0) {
            return 0.0;
        }
        return std::chrono::duration<double, std::milli>(total).count() /
               static_cast<double>(samples);
    }

    double maxMilliseconds() const {
        return std::chrono::duration<double, std::milli>(max).count();
    }
};

class ScopedTimer {
public:
    explicit ScopedTimer(TimingStats& stats)
        : _stats(enabled() ? &stats : nullptr) {
        if (_stats != nullptr) {
            _start = TimingStats::clock::now();
        } else {
            _active = false;
        }
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    ScopedTimer& operator=(ScopedTimer&&) = delete;

    ~ScopedTimer() {
        stop();
    }

    void stop() {
        if (!_active) {
            return;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            TimingStats::clock::now() - _start);
        _stats->add(elapsed);
        _active = false;
    }

private:
    TimingStats* _stats{nullptr};
    TimingStats::clock::time_point _start;
    bool _active{true};
};

inline std::string formatTimingStats(const std::string& label, const TimingStats& stats) {
    if (!enabled()) {
        return label + ": profiling disabled";
    }

    std::ostringstream out;
    out << label << ": avg " << std::fixed << std::setprecision(3)
        << stats.averageMilliseconds() << " ms, max " << stats.maxMilliseconds() << " ms"
        << " (" << stats.samples << " samples)";
    return out.str();
}

}  // namespace profiling

#endif  // UTILITIES_TIMING_H
