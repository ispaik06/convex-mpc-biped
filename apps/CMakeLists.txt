add_executable(main
	main.cpp
	main_helper.cpp
)

target_link_libraries(main PRIVATE
	robot
	sim
	My_Controller
)
