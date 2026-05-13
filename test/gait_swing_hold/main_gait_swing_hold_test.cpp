#include <cstdlib>
#include <cctype>
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
              << "\t robot-id: m for MIT humanoid, g for Unitree G1, h for Unitree H1\n"
              << "\t Viewer: y for GUI, n for headless\n"
              << "\t torso-z-offset-m: optional base height offset in meters, default 0.0\n";
}

bool robotTypeFromId(const char robotId, RobotType& robotType) {
    switch (std::tolower(static_cast<unsigned char>(robotId))) {
        case 'm':
            robotType = RobotType::MIT_HUMANOID;
            return true;
        case 'g':
            robotType = RobotType::UNITREE_G1;
            return true;
        case 'h':
            robotType = RobotType::UNITREE_H1;
            return true;
        default:
            return false;
    }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        printUsage();
        return EXIT_FAILURE;
    }

    RobotType robotType = RobotType::MIT_HUMANOID;
    if (!robotTypeFromId(argv[1][0], robotType)) {
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

    setActiveRobotType(robotType);

    auto controller = std::make_unique<GaitSwingHoldController>();
    GaitSwingHoldTestRunner runner(robotType, controller.get(), headless, torsoZOffset);
    runner.init();
    runner.run();
    return EXIT_SUCCESS;
}
