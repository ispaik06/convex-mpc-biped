#include <iostream>

#include "SimViewer.h"

SimViewer::~SimViewer() {
	shutdown();
}

bool SimViewer::init(const mjModel* model) {
	shutdown();

	if (!glfwInit()) {
		std::cerr << "GLFW initialization failed, continuing without viewer.\n";
		return false;
	}

	glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

	_window = glfwCreateWindow(1400, 900, "ConvexMPC MuJoCo Viewer", nullptr, nullptr);
	if (!_window) {
		std::cerr << "GLFW window creation failed, continuing without viewer.\n";
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(_window);
	glfwSwapInterval(1);

	mjv_defaultCamera(&_camera);
	mjv_defaultFreeCamera(model, &_camera);
	mjv_defaultOption(&_option);
	mjv_defaultScene(&_scene);
	mjr_defaultContext(&_context);

	mjv_makeScene(model, &_scene, 4000);
	mjr_makeContext(model, &_context, mjFONTSCALE_150);

	_enabled = true;
	return true;
}


void SimViewer::shutdown() {
	if (!_enabled) {
		return;
	}

	mjr_freeContext(&_context);
	mjv_freeScene(&_scene);

	if (_window) {
		glfwDestroyWindow(_window);
		_window = nullptr;
	}

	glfwTerminate();
	_enabled = false;
}


void SimViewer::render(const mjModel* model, mjData* data) {
	if (!_enabled) {
		return;
	}

	mjrRect viewport{0, 0, 0, 0};
	glfwGetFramebufferSize(_window, &viewport.width, &viewport.height);

	mjv_updateScene(model,
					data,
					&_option,
					nullptr,
					&_camera,
					mjCAT_ALL,
					&_scene);
	mjr_render(viewport, &_scene, &_context);

	glfwSwapBuffers(_window);
	glfwPollEvents();
}

bool SimViewer::isEnabled() const {
	return _enabled;
}

bool SimViewer::shouldClose() const {
	return _enabled && glfwWindowShouldClose(_window);
}
