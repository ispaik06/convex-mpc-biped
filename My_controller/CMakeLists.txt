add_library(My_Controller
	src/My_Controller.cpp
	src/ControlParameters.cpp
	src/ControlFSM.cpp
	src/ReferenceTrajectory.cpp
	src/MPCFormulation.cpp
	src/ConvexMPC.cpp
	# src/GaitScheduler.cpp
	# src/ControlFSM.cpp
)

if(NOT DEFINED osqp_DIR)
	set(osqp_DIR "$ENV{HOME}/.local/osqp/lib/cmake/osqp")
endif()

if(NOT OsqpEigen_DIR OR OsqpEigen_DIR STREQUAL "OsqpEigen_DIR-NOTFOUND")
	set(OsqpEigen_DIR "$ENV{HOME}/.local/osqp-eigen/lib/cmake/OsqpEigen" CACHE PATH "Path to OsqpEigenConfig.cmake" FORCE)
endif()

find_package(osqp CONFIG REQUIRED)
find_package(OsqpEigen REQUIRED)


target_include_directories(My_Controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/MyController
	$<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(My_Controller PUBLIC
	common
	robot
	osqp::osqp
	OsqpEigen::OsqpEigen
)
