#ifndef MAIN_THREAD_H
#define MAIN_THREAD_H

#include <memory>
#include <string>

struct mjModel_;
struct mjData_;
struct mjvCamera_;
struct mjvOption_;
struct mjvPerturb_;

namespace mujoco {
class Simulate;
}

class MainThread {
public:
	MainThread();
	~MainThread();

	MainThread(const MainThread&) = delete;
	MainThread& operator=(const MainThread&) = delete;

	void init();
	void load(mjModel_* model, mjData_* data, const std::string& displayed_filename);
	void run();
	void sync(bool state_only = false);
	void requestExit();
	bool exitRequested() const;

private:
	void ensureInitialized();

	std::unique_ptr<mujoco::Simulate> _simulate;
	std::unique_ptr<mjvCamera_> _camera;
	std::unique_ptr<mjvOption_> _option;
	std::unique_ptr<mjvPerturb_> _perturb;
};

#endif  // MAIN_THREAD_H
