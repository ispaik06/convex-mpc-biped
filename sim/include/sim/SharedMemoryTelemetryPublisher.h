#ifndef SHARED_MEMORY_TELEMETRY_PUBLISHER_H
#define SHARED_MEMORY_TELEMETRY_PUBLISHER_H

#include <cstddef>
#include <cstdint>
#include <string>

#include <mujoco/mujoco.h>

#include "Types.h"

class SharedMemoryTelemetryPublisher {
public:
    static constexpr std::size_t kQposCapacity = 256;

    explicit SharedMemoryTelemetryPublisher(std::string robot_name = {},
                                            std::string model_xml_path = {},
                                            std::string shared_memory_name = {});
    ~SharedMemoryTelemetryPublisher();

    SharedMemoryTelemetryPublisher(const SharedMemoryTelemetryPublisher&) = delete;
    SharedMemoryTelemetryPublisher& operator=(const SharedMemoryTelemetryPublisher&) = delete;

    void publish(u64 iteration,
                 double sim_time,
                 const mjtNum* qpos,
                 std::size_t qpos_len);

    bool enabled() const noexcept { return _layout != nullptr; }
    const std::string& sharedMemoryName() const noexcept { return _sharedMemoryName; }

private:
    struct SharedMemoryLayout {
        std::uint64_t sequence{0};
        std::uint64_t iteration{0};
        double sim_time{0.0};
        char robot_name[32]{};
        char model_xml_path[256]{};
        std::uint32_t version{1};
        std::uint32_t qpos_dim{0};
        double qpos[kQposCapacity]{};
        std::uint64_t reserved0{0};
        std::uint64_t reserved1{0};
    };
    static_assert(sizeof(SharedMemoryLayout) == 2384, "Unexpected viewer shared memory size");

    static std::string defaultSharedMemoryName();
    static std::string normalizedPosixName(const std::string& name);
    static void writeString(char* destination, std::size_t capacity, const std::string& value);

    void initializeSharedMemory();

    std::string _sharedMemoryName;
    std::string _posixName;
    std::string _robotName;
    std::string _modelXmlPath;
    int _fd{-1};
    SharedMemoryLayout* _layout{nullptr};
};

#endif  // SHARED_MEMORY_TELEMETRY_PUBLISHER_H
