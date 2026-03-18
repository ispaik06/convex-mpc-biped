add_library(My_controller INTERFACE)

target_include_directories(My_controller INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/include/controller
	$<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)
