#include <cassert>
#include <iostream>

#include "main_helper.h"
#include "SimulationRunner.h"
#include "Types.h"


void printUsage() {
	printf(
		"Usage: robot [robot-id] \n"
		"\t where robot-id:    m for MIT humanoid, g for UNITREE G1\n");
}

int main_helper(int argc, char** argv, RobotController* ctrl) {
	if(argc > 3 || argc < 1) {
		printUsage();
		return EXIT_FAILURE;
	}


	if(argv[1][0] == 'm') {
		SimulationRunner simulationRunner(RobotType::MIT_HUMANOID, ctrl);
		simulationRunner.init();
		simulationRunner.run();
		printf("[MIT HUMANOID] Sim Runner run() has finished!\n");
	}
	else if(argv[1][0] == 'g') {
		SimulationRunner simulationRunner(RobotType::UNITREE_G1, ctrl);
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
