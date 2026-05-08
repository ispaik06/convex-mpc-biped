add_executable(stand_contact_probe
    stand_contact_probe.cpp
)

add_executable(stand_rh_probe
    stand_rh_probe.cpp
)

add_executable(wrench_mapping_probe
    wrench_mapping_probe.cpp
)

# Manual debug targets. Intentionally not added to CTest because they depend on
# standing_mpc_debug_*.json or walking_mpc_debug_*.json snapshots and produce
# diagnostic reports/plots.

target_include_directories(stand_contact_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(stand_contact_probe PRIVATE
    sim
    My_Controller
    nlohmann_json::nlohmann_json
)

target_compile_definitions(stand_contact_probe PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

target_include_directories(stand_rh_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(stand_rh_probe PRIVATE
    My_Controller
    nlohmann_json::nlohmann_json
)

target_compile_definitions(stand_rh_probe PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

target_include_directories(wrench_mapping_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(wrench_mapping_probe PRIVATE
    sim
    nlohmann_json::nlohmann_json
)

target_compile_definitions(wrench_mapping_probe PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
