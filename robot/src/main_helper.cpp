#include <cassert>
#include <iostream>

#include "main_helper.h"
#include "SimulationRunner.h"
#include "Types.h"


void printUsage() {
	printf(
		"Usage: robot [robot-id] Viewer [y/n] \n"
		"\t where robot-id:    m for MIT humanoid, g for UNITREE G1\n"
		"\t where Viewer  :    y for opening viewer, n for headless mode\n");
}

int main_helper(int argc, char** argv, RobotController* ctrl) {
	bool headless;

	if(argc > 4 || argc == 1) {
		printUsage();
		return EXIT_FAILURE;
	}
	else if(argc == 2) {
		if(argv[1][0] == 'm' || argv[1][0] == 'g') headless = true;
		else {
			printUsage();
			return EXIT_FAILURE;
		}
	}

	if(argv[2][0] == 'y' || argv[2][0] == 'Y') {
		headless = false;
	}
	else if(argv[2][0] == 'n' || argv[2][0] == 'N') {
		headless = true;
	}
	else {
		printUsage();
		return EXIT_FAILURE;
	}

	if(argv[1][0] == 'm') {
		SimulationRunner simulationRunner(RobotType::MIT_HUMANOID, ctrl, headless);
		simulationRunner.init();
		simulationRunner.run();
		printf("[MIT HUMANOID] Sim Runner run() has finished!\n");
	}
	else if(argv[1][0] == 'g') {
		SimulationRunner simulationRunner(RobotType::MIT_HUMANOID, ctrl, headless);
		simulationRunner.init();
		simulationRunner.run();
		printf("[MIT HUMANOID] Sim Runner run() has finished!\n");
	}
	else {
		printUsage();
		return EXIT_FAILURE;
	}

	return 0;
}
