#include <cassert>
#include <iostream>

#include "main_helper.h"
#include "SimulationRunner.h"
#include "Types.h"


void printUsage() {
	printf(
		"Usage: robot [robot-id] Viewer [y/n] \n"
		"\t where robot-id:    m for MIT humanoid, g for UNITREE G1\n"
		"\t where default viewer mode  :    GUI viewer");
}

int main_helper(int argc, char** argv, RobotController* ctrl) {
	bool headless = false;

	if(argc > 3 || argc == 1) {
		printUsage();
		return EXIT_FAILURE;
	}
	if(argc == 2) {
		// TODO: Revisit the single-argument CLI once the non-MIT robot flows are
		// finalized. For now, the existing personal workflow keeps this narrow.
		if(argv[1][0] == 'm' || argv[1][0] == 'g') headless = false;
		else {
			printUsage();
			return EXIT_FAILURE;
		}
	}

	if(argc == 3) {
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
	}

	if(argv[1][0] == 'm') {
		SimulationRunner simulationRunner(RobotType::MIT_HUMANOID, ctrl, headless);
		simulationRunner.init();
		simulationRunner.run();
		printf("[MIT HUMANOID] Sim Runner run() has finished!\n");
	}
	else if(argv[1][0] == 'g') {
		SimulationRunner simulationRunner(RobotType::UNITREE_G1, ctrl, headless);
		simulationRunner.init();
		simulationRunner.run();
		printf("[UNITREE G1] Sim Runner run() has finished!\n");
	}
	else if(argv[1][0] == 'h') {
		SimulationRunner simulationRunner(RobotType::UNITREE_H1, ctrl, headless);
		simulationRunner.init();
		simulationRunner.run();
		printf("[UNITREE H1] Sim Runner run() has finished!\n");
	}
	else {
		printUsage();
		return EXIT_FAILURE;
	}

	return 0;
}
