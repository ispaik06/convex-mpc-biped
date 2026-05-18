#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>

#include "RobotConfig.h"
#include "SharedMemoryTelemetryPublisher.h"
#include "Types.h"
#include "Utilities/Timing.h"

namespace {

struct MjModelDeleter {
    void operator()(mjModel* model) const {
        if (model != nullptr) {
            mj_deleteModel(model);
        }
    }
};

double publishHz() {
    const char* hzEnv = std::getenv("CONVEXMPC_ZERO_COMMAND_HZ");
    if (hzEnv != nullptr && hzEnv[0] != '\0') {
        const double hz = std::atof(hzEnv);
        if (std::isfinite(hz) && hz > 0.0) {
            return hz;
        }
    }
    return 60.0;
}

void printUsage() {
    std::printf(
        "Usage: main_zero [robot-id]\n"
        "\t robot-id: m for MIT humanoid\n"
        "\t Publishes nominal qpos only; no controller or native viewer is started.\n");
}

}  // namespace

int main(int argc, char** argv) {
    profiling::configureFromEnvironment();

    if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
        printUsage();
        return (argc > 2) ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    const char robotId = (argc == 2) ? argv[1][0] : 'm';
    if (!(robotId == 'm' || robotId == 'M')) {
        std::fprintf(stderr, "[main_zero] only MIT humanoid is supported for this viewer test\n");
        return EXIT_FAILURE;
    }

    setActiveRobotType(RobotType::MIT_HUMANOID);
    const auto& runtimeConfig = getRobotRuntimeConfig(RobotType::MIT_HUMANOID);
    const std::string modelPath = resolveProjectPath(runtimeConfig.modelXmlPath);

    if (mjVERSION_HEADER != mj_version()) {
        throw std::runtime_error("MuJoCo header/library version mismatch");
    }

    std::array<char, 1024> error{};
    std::unique_ptr<mjModel, MjModelDeleter> model(
        mj_loadXML(modelPath.c_str(), nullptr, error.data(), error.size()));
    if (model == nullptr) {
        throw std::runtime_error("mj_loadXML failed for " + modelPath + ": " + error.data());
    }

    SharedMemoryTelemetryPublisher publisher("MIT Humanoid", modelPath);
    if (!publisher.enabled()) {
        std::cerr << "[main_zero] qpos shared memory publisher is disabled\n";
    }

    const double hz = publishHz();
    const auto period = std::chrono::duration<double>(1.0 / hz);
    auto nextTick = std::chrono::steady_clock::now();
    u64 iteration = 0;
    double simTime = 0.0;

    std::cout << "[main_zero] loaded " << modelPath << '\n'
              << "[main_zero] qpos shared memory: " << publisher.sharedMemoryName() << '\n'
              << "[main_zero] publishing nominal qpos at " << hz << " Hz\n";

    while (true) {
        publisher.publish(iteration,
                          simTime,
                          model->qpos0,
                          static_cast<std::size_t>(model->nq));
        ++iteration;
        simTime += period.count();
        nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(nextTick);
    }

    return EXIT_SUCCESS;
}
