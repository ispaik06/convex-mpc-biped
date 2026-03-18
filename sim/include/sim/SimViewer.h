#ifndef SIM_VIEWER_H
#define SIM_VIEWER_H

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

class SimViewer {
public:
	SimViewer() = default;
	SimViewer(const SimViewer&) = delete;
	SimViewer& operator=(const SimViewer&) = delete;
	~SimViewer();

	bool init(const mjModel* model);
	void render(const mjModel* model, mjData* data);
	void shutdown();
	bool isEnabled() const;
	bool shouldClose() const;

private:
	GLFWwindow* _window = nullptr;
	mjvCamera _camera{};
	mjvOption _option{};
	mjvScene _scene{};
	mjrContext _context{};
	bool _enabled = false;
};


#endif  // SIM_VIEWER_H
