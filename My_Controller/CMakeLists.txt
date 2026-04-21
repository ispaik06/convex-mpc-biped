add_library(My_Controller
	src/My_Controller.cpp
	src/ControllerConfig.cpp
	src/GaitScheduler.cpp
	src/ControlFSM.cpp
	src/ReferenceTrajectory.cpp
	src/SwingFootTrajectory.cpp
	src/MPCFormulation.cpp
	src/ConvexMPC.cpp
	src/StandingMpcDebugLogger.cpp
)

find_package(osqp CONFIG REQUIRED)
find_package(OsqpEigen REQUIRED)

find_path(NLOHMANN_JSON_INCLUDE_DIR
	NAMES nlohmann/json.hpp
	HINTS
		/opt/homebrew/opt/nlohmann-json/include
		/usr/local/opt/nlohmann-json/include
	REQUIRED
)


target_include_directories(My_Controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/MyController
	$<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_include_directories(My_Controller PRIVATE
	${NLOHMANN_JSON_INCLUDE_DIR}
)

target_link_libraries(My_Controller PUBLIC
	common
	robot
	osqp::osqp
	OsqpEigen::OsqpEigen
)

target_compile_definitions(My_Controller
	PRIVATE
		PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
