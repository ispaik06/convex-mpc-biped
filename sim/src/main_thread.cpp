#include <memory>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

#include "glfw_adapter.h"
#include "simulate.h"

#include "main_thread.h"

MainThread::MainThread() = default;

MainThread::~MainThread() {
	requestExit();
}

void MainThread::ensureInitialized() {
	if (_simulate) {
		return;
	}

	_camera = std::make_unique<mjvCamera>();
	_option = std::make_unique<mjvOption>();
	_perturb = std::make_unique<mjvPerturb>();

	mjv_defaultCamera(_camera.get());
	mjv_defaultOption(_option.get());
	// Keep geom group 4 visible at startup so debug / helper geoms do not need
	// to be enabled manually in the viewer every run.
	_option->geomgroup[4] = 1;
	mjv_defaultPerturb(_perturb.get());

	_simulate = std::make_unique<mujoco::Simulate>(
		std::make_unique<mujoco::GlfwAdapter>(),
		_camera.get(),
		_option.get(),
		_perturb.get(),
		true);
}

void MainThread::init() {
	ensureInitialized();
}

void MainThread::load(mjModel_* model, mjData_* data, const std::string& displayed_filename) {
	if (!_simulate) {
		throw std::runtime_error("MainThread::init must be called before load");
	}
	_simulate->Load(reinterpret_cast<mjModel*>(model),
				    reinterpret_cast<mjData*>(data),
				    displayed_filename.c_str());
}

void MainThread::run() {
	ensureInitialized();
	_simulate->RenderLoop();
}

void MainThread::sync(bool state_only) {
	if (_simulate) {
		_simulate->Sync(state_only);
	}
}

void MainThread::requestExit() {
	if (_simulate) {
		_simulate->exitrequest.store(1);
	}
}

bool MainThread::exitRequested() const {
	return _simulate && _simulate->exitrequest.load();
}
