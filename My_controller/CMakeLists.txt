add_library(My_Controller
	src/My_Controller.cpp
	src/ControlFSM.cpp
	src/ReferenceTrajectory.cpp
	# src/GaitScheduler.cpp
	# src/ControlFSM.cpp
)

target_include_directories(My_Controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include/MyController
	$<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(My_Controller PUBLIC
	common
	robot
)
