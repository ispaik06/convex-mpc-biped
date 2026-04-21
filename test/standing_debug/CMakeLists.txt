add_executable(stand_contact_probe
    stand_contact_probe.cpp
)

# Manual debug target. Intentionally not added to CTest because it depends on a
# standing_mpc_debug_*.json snapshot and reports diagnostic contact wrenches.

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
