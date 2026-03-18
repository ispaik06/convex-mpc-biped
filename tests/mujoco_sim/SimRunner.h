#ifndef SIM_RUNNER_H
#define SIM_RUNNER_H

#include <string>

#include "Types.h"

class SimuRunner {
public:
	explicit SimuRunner(std::string modelPath = {});

	void run();
	void setModelPath(std::string modelPath);

private:
	std::string _modelPath;
	u64 _iteration = 0;
};

#endif  // SIM_RUNNER_H
