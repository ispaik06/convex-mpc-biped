#include "SharedMemoryPublisher.h"

#include <cerrno>
#include <cstdlib>
#include <cstddef>
#include <cstdio>
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

void closeFd(int& fd) {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}
}  // namespace

DashboardSharedMemoryPublisher::DashboardSharedMemoryPublisher(std::string robot_name,
                                                               std::string shared_memory_name)
    : _sharedMemoryName(shared_memory_name.empty() ? defaultSharedMemoryName()
                                                   : std::move(shared_memory_name)),
      _posixName(normalizedPosixName(_sharedMemoryName)),
      _robotName(robot_name.empty() ? std::string{} : std::move(robot_name)) {
    initializeSharedMemory();
}

DashboardSharedMemoryPublisher::~DashboardSharedMemoryPublisher() {
    if (_layout != nullptr) {
        munmap(_layout, sizeof(SharedMemoryLayout));
        _layout = nullptr;
    }
    closeFd(_fd);
}

std::string DashboardSharedMemoryPublisher::defaultSharedMemoryName() {
    const char* envName = std::getenv("CONVEXMPC_SHM_NAME");
    if (envName != nullptr && envName[0] != '\0') {
        return std::string(envName);
    }
    return "convexmpc_dashboard_state";
}

std::string DashboardSharedMemoryPublisher::normalizedPosixName(const std::string& name) {
    if (name.empty()) {
        return "/convexmpc_dashboard_state";
    }
    if (name.front() == '/') {
        return name;
    }
    return "/" + name;
}

void DashboardSharedMemoryPublisher::writeString(char* destination,
                                                 const std::size_t capacity,
                                                 const std::string& value) {
    if (capacity == 0) {
        return;
    }

    std::snprintf(destination, capacity, "%s", value.c_str());
}

void DashboardSharedMemoryPublisher::initializeSharedMemory() {
    _fd = shm_open(_posixName.c_str(), O_CREAT | O_RDWR, 0600);
    if (_fd < 0) {
        std::cerr << "[DashboardSharedMemory] shm_open failed for " << _posixName
                  << ": " << std::strerror(errno) << std::endl;
        return;
    }

    if (ftruncate(_fd, static_cast<off_t>(sizeof(SharedMemoryLayout))) != 0) {
        std::cerr << "[DashboardSharedMemory] ftruncate failed for " << _posixName
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
        std::cerr << "[DashboardSharedMemory] mmap failed for " << _posixName
                  << ": " << std::strerror(errno) << std::endl;
        closeFd(_fd);
        return;
    }

    _layout = reinterpret_cast<SharedMemoryLayout*>(mapped);
    std::memset(_layout, 0, sizeof(SharedMemoryLayout));
    _layout->version = 2;
    _layout->state_dim = static_cast<std::uint32_t>(kStateDim);
    writeString(_layout->robot_name, kRobotNameCapacity, _robotName);

    closeFd(_fd);
}

void DashboardSharedMemoryPublisher::publish(
    const u64 iteration,
    const double sim_time,
    const std::array<double, kStateDim>& state) {
    if (_layout == nullptr) {
        return;
    }

    __atomic_fetch_add(&_layout->sequence, 1ULL, __ATOMIC_RELEASE);
    _layout->iteration = static_cast<std::uint64_t>(iteration);
    _layout->sim_time = sim_time;
    std::memcpy(_layout->state, state.data(), sizeof(double) * kStateDim);
    __atomic_thread_fence(__ATOMIC_RELEASE);
    __atomic_fetch_add(&_layout->sequence, 1ULL, __ATOMIC_RELEASE);
}
