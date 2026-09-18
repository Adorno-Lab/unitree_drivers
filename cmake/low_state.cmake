# ---------------------------------------------------------------------------
# Library: low_state (wraps DriverUnitreeLowState)
# ---------------------------------------------------------------------------
# No sas_core dependency at all (no background control loop -- see the class's
# own docs), so this target has no shared/static constraint forcing SHARED the
# way loco_client/g1_arm_sdk do. Built SHARED anyway for consistency across the
# package. Eigen3 is PUBLIC (Eigen/Dense is in the public header); unitree_sdk2
# is only used in the .cpp, so it's PRIVATE here.
add_library(low_state SHARED
    src/DriverUnitreeLowState.cpp
)
add_library(unitree_drivers::low_state ALIAS low_state)

target_compile_features(low_state PUBLIC cxx_std_17)

target_include_directories(low_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(low_state
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        unitree_sdk2
)

set_target_properties(low_state PROPERTIES
    OUTPUT_NAME unitree_drivers_low_state
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
