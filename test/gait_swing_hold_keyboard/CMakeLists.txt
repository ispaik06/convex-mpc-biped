add_executable(main_gait_swing_hold_keyboard_test
    main_gait_swing_hold_keyboard_test.cpp
    KeyboardGaitSwingRunner.cpp
    ${PROJECT_SOURCE_DIR}/test/gait_swing_hold/GaitSwingHoldController.cpp
)

# Manual inspection target. Intentionally not added to CTest.

target_include_directories(main_gait_swing_hold_keyboard_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/test/gait_swing_hold
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(main_gait_swing_hold_keyboard_test PRIVATE
    sim
    My_Controller
)

target_compile_definitions(main_gait_swing_hold_keyboard_test PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
