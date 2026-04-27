add_executable(main_left_swing_hold_test
    main_left_swing_hold_test.cpp
    LeftSwingHoldController.cpp
    LeftSwingHoldTestRunner.cpp
)

# Manual inspection target. Intentionally not added to CTest.

target_include_directories(main_left_swing_hold_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(main_left_swing_hold_test PRIVATE
    sim
    My_Controller
)

target_compile_definitions(main_left_swing_hold_test PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
