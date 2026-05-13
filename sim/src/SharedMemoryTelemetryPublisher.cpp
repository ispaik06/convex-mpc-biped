#include "SharedMemoryTelemetryPublisher.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {
constexpr std::size_t kRobotNameCapacity = 32;
constexpr std::size_t kModelXmlPathCapacity = 256;

void closeFd(int& fd) {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}
}  // namespace

SharedMemoryTelemetryPublisher::SharedMemoryTelemetryPublisher(std::string robot_name,
                                                               std::string model_xml_path,
                                                               std::string shared_memory_name)
    : _sharedMemoryName(shared_memory_name.empty() ? defaultSharedMemoryName()
                                                   : std::move(shared_memory_name)),
      _posixName(normalizedPosixName(_sharedMemoryName)),
      _robotName(robot_name.empty() ? std::string{} : std::move(robot_name)),
      _modelXmlPath(model_xml_path.empty() ? std::string{} : std::move(model_xml_path)) {
    initializeSharedMemory();
}

SharedMemoryTelemetryPublisher::~SharedMemoryTelemetryPublisher() {
    if (_layout != nullptr) {
        munmap(_layout, sizeof(SharedMemoryLayout));
        _layout = nullptr;
    }
    closeFd(_fd);
}

std::string SharedMemoryTelemetryPublisher::defaultSharedMemoryName() {
    const char* envName = std::getenv("CONVEXMPC_QPOS_SHM_NAME");
    if (envName != nullptr && envName[0] != '\0') {
        return std::string(envName);
    }
    return "convexmpc_dashboard_qpos";
}

std::string SharedMemoryTelemetryPublisher::normalizedPosixName(const std::string& name) {
    if (name.empty()) {
        return "/convexmpc_dashboard_qpos";
    }
    if (name.front() == '/') {
        return name;
    }
    return "/" + name;
}

void SharedMemoryTelemetryPublisher::writeString(char* destination,
                                                 const std::size_t capacity,
                                                 const std::string& value) {
    if (capacity == 0) {
        return;
    }

    std::snprintf(destination, capacity, "%s", value.c_str());
}

void SharedMemoryTelemetryPublisher::initializeSharedMemory() {
    _fd = shm_open(_posixName.c_str(), O_CREAT | O_RDWR, 0600);
    if (_fd < 0) {
        std::cerr << "[SharedMemoryTelemetryPublisher] shm_open failed for " << _posixName
                  << ": " << std::strerror(errno) << std::endl;
        return;
    }

    if (ftruncate(_fd, static_cast<off_t>(sizeof(SharedMemoryLayout))) != 0) {
        std::cerr << "[SharedMemoryTelemetryPublisher] ftruncate failed for " << _posixName
                  << ": " << std::strerror(errno) << std::endl;
        closeFd(_fd);
        return;
    }

    void* mapped = mmap(nullptr,
                        sizeof(SharedMemoryLayout),
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED,
                        _fd,
                        0);
    if (mapped == MAP_FAILED) {
        std::cerr << "[SharedMemoryTelemetryPublisher] mmap failed for " << _posixName
                  << ": " << std::strerror(errno) << std::endl;
        closeFd(_fd);
        return;
    }

    _layout = reinterpret_cast<SharedMemoryLayout*>(mapped);
    std::memset(_layout, 0, sizeof(SharedMemoryLayout));
    _layout->version = 1;
    writeString(_layout->robot_name, kRobotNameCapacity, _robotName);
    writeString(_layout->model_xml_path, kModelXmlPathCapacity, _modelXmlPath);

    closeFd(_fd);
}

void SharedMemoryTelemetryPublisher::publish(const u64 iteration,
                                             const double sim_time,
                                             const mjtNum* qpos,
                                             const std::size_t qpos_len) {
    if (_layout == nullptr || qpos == nullptr) {
        return;
    }

    const std::size_t copy_count = std::min(qpos_len, kQposCapacity);

    __atomic_fetch_add(&_layout->sequence, 1ULL, __ATOMIC_RELEASE);
    _layout->iteration = static_cast<std::uint64_t>(iteration);
    _layout->sim_time = sim_time;
    _layout->qpos_dim = static_cast<std::uint32_t>(copy_count);
    for (std::size_t i = 0; i < copy_count; ++i) {
        _layout->qpos[i] = static_cast<double>(qpos[i]);
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_fetch_add(&_layout->sequence, 1ULL, __ATOMIC_RELEASE);
}
