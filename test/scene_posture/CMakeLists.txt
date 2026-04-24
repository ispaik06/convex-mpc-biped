add_executable(scene_posture_clearance
    posture_clearance.cpp
)

# Manual inspection target. This is meant to be run directly against a scene
# XML and is intentionally not added to CTest.

target_link_libraries(scene_posture_clearance PRIVATE
    mujoco::mujoco
)

target_compile_definitions(scene_posture_clearance PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
