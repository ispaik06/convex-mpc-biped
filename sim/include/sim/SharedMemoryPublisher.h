#ifndef DASHBOARD_SHARED_MEMORY_PUBLISHER_H
#define DASHBOARD_SHARED_MEMORY_PUBLISHER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "Types.h"

class DashboardSharedMemoryPublisher {
public:
    static constexpr std::size_t kStateDim = 12;

    explicit DashboardSharedMemoryPublisher(std::string robot_name = {},
                                            std::string shared_memory_name = {});
    ~DashboardSharedMemoryPublisher();

    DashboardSharedMemoryPublisher(const DashboardSharedMemoryPublisher&) = delete;
    DashboardSharedMemoryPublisher& operator=(const DashboardSharedMemoryPublisher&) = delete;

    void publish(u64 iteration, double sim_time, const std::array<double, kStateDim>& state);

    bool enabled() const noexcept { return _layout != nullptr; }
    const std::string& sharedMemoryName() const noexcept { return _sharedMemoryName; }

private:
    struct SharedMemoryLayout {
        std::uint64_t sequence{0};
        std::uint64_t iteration{0};
        double sim_time{0.0};
        char robot_name[32]{};
        double state[kStateDim]{};
        std::uint32_t version{1};
        std::uint32_t state_dim{static_cast<std::uint32_t>(kStateDim)};
        std::uint64_t reserved0{0};
        std::uint64_t reserved1{0};
    };
    static_assert(sizeof(SharedMemoryLayout) == 176, "Unexpected dashboard shared memory size");

    static std::string defaultSharedMemoryName();
    static std::string normalizedPosixName(const std::string& name);
    static void writeString(char* destination, std::size_t capacity, const std::string& value);

    void initializeSharedMemory();

    std::string _sharedMemoryName;
    std::string _posixName;
    std::string _robotName;
    int _fd{-1};
    SharedMemoryLayout* _layout{nullptr};
};

#endif  // DASHBOARD_SHARED_MEMORY_PUBLISHER_H
