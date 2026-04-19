#include <cstdlib>
#include <iostream>
#include <memory>

#include "LeftSwingHoldController.h"
#include "LockedTorsoSwingRunner.h"

namespace {
void printUsage() {
    std::cout << "Usage: main_left_swing_hold_test [robot-id] Viewer [y/n]\n"
              << "\t robot-id: m for MIT humanoid\n"
              << "\t Viewer: y for GUI, n for headless\n";
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        printUsage();
        return EXIT_FAILURE;
    }

    if (argv[1][0] != 'm') {
        printUsage();
        return EXIT_FAILURE;
    }

    bool headless = false;
    if (argc == 3) {
        if (argv[2][0] == 'n' || argv[2][0] == 'N') {
            headless = true;
        } else if (argv[2][0] == 'y' || argv[2][0] == 'Y') {
            headless = false;
        } else {
            printUsage();
            return EXIT_FAILURE;
        }
    }

    auto controller = std::make_unique<LeftSwingHoldController>();
    LockedTorsoSwingRunner runner(RobotType::MIT_HUMANOID, controller.get(), headless);
    runner.init();
    runner.run();
    return EXIT_SUCCESS;
}
