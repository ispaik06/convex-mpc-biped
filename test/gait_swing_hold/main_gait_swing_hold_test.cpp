#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "GaitSwingHoldController.h"
#include "GaitSwingHoldTestRunner.h"
#include "RobotConfig.h"

namespace {
void printUsage() {
    std::cout << "Usage: main_gait_swing_hold_test [robot-id] Viewer [y/n] [torso-z-offset-m]\n"
              << "\t robot-id: m for MIT humanoid\n"
              << "\t Viewer: y for GUI, n for headless\n"
              << "\t torso-z-offset-m: optional base height offset in meters, default 0.0\n";
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        printUsage();
        return EXIT_FAILURE;
    }

    if (argv[1][0] != 'm') {
        printUsage();
        return EXIT_FAILURE;
    }

    bool headless = false;
    if (argc >= 3) {
        if (argv[2][0] == 'n' || argv[2][0] == 'N') {
            headless = true;
        } else if (argv[2][0] == 'y' || argv[2][0] == 'Y') {
            headless = false;
        } else {
            printUsage();
            return EXIT_FAILURE;
        }
    }

    double torsoZOffset = 0.0;
    if (argc == 4) {
        try {
            torsoZOffset = std::stod(argv[3]);
        } catch (const std::exception&) {
            printUsage();
            return EXIT_FAILURE;
        }
    }

    setActiveRobotType(RobotType::MIT_HUMANOID);

    auto controller = std::make_unique<GaitSwingHoldController>();
    GaitSwingHoldTestRunner runner(RobotType::MIT_HUMANOID, controller.get(), headless, torsoZOffset);
    runner.init();
    runner.run();
    return EXIT_SUCCESS;
}
