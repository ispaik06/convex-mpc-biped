add_executable(main
	main.cpp
	main_helper.cpp
)

target_link_libraries(main PRIVATE
	sim
	My_Controller
)
