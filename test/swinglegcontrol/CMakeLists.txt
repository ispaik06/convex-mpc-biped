add_executable(main_swing_test
    main_swing_test.cpp
    SwingLegTestController.cpp
    FixedBaseSwingTestRunner.cpp
)

# Intentionally kept out of CTest. This target is for manual verification only.

target_include_directories(main_swing_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(main_swing_test PRIVATE
    common
    robot
    sim
    My_Controller
)

target_compile_definitions(main_swing_test PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
