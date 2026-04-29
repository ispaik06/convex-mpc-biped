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

find_path(NLOHMANN_JSON_INCLUDE_DIR
    NAMES nlohmann/json.hpp
    HINTS
        /opt/homebrew/opt/nlohmann-json/include
        /usr/local/opt/nlohmann-json/include
    REQUIRED
)

target_include_directories(stand_contact_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    ${NLOHMANN_JSON_INCLUDE_DIR}
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(stand_contact_probe PRIVATE
    sim
    My_Controller
)

target_compile_definitions(stand_contact_probe PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

target_include_directories(stand_rh_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/My_Controller/include/MyController
    ${NLOHMANN_JSON_INCLUDE_DIR}
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(stand_rh_probe PRIVATE
    My_Controller
)

target_compile_definitions(stand_rh_probe PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

target_include_directories(wrench_mapping_probe PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
    $<TARGET_PROPERTY:sim,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:robot,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(wrench_mapping_probe PRIVATE
    sim
)

target_compile_definitions(wrench_mapping_probe PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)
