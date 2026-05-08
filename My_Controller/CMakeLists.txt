add_library(My_Controller
	src/My_Controller.cpp
	src/ContactManager.cpp
	src/ControllerConfig.cpp
	src/GaitScheduler.cpp
	src/LocomotionFSM.cpp
	src/SwingFootPlanner.cpp
	src/ReferenceTrajectory.cpp
	src/SwingFootTrajectory.cpp
	src/MPCFormulation.cpp
	src/ConvexMPC.cpp
	src/StandingMpcDebugLogger.cpp
)

find_package(osqp CONFIG REQUIRED)
find_package(OsqpEigen CONFIG REQUIRED)


target_include_directories(My_Controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/MyController
	$<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(My_Controller PUBLIC
	common
	robot
	OsqpEigen::OsqpEigen
)

target_link_libraries(My_Controller PRIVATE
	$<IF:$<TARGET_EXISTS:osqp::osqp>,osqp::osqp,osqp::osqpstatic>
	nlohmann_json::nlohmann_json
)

target_compile_definitions(My_Controller
	PRIVATE
		PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
