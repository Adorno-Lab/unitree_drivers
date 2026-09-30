# ---------------------------------------------------------------------------
# Library: arm_sdk (wraps DriverUnitreeArmSDK)
# ---------------------------------------------------------------------------
# No dqrobotics/Eigen dependency: DriverUnitreeArmSDK.h only pulls in <array>,
# <memory>, <sas_core/sas_shutdown_signaler.hpp>. unitree_sdk2 is used only in
# the .cpp, but is kept PUBLIC here anyway since it's the shared vocabulary type
# of this whole package family (consistent with loco_client) -- feel free to
# make it PRIVATE if that's not a concern for your consumers.

add_library(arm_sdk SHARED
    src/DriverUnitreeArmSDK.cpp
)
add_library(unitree_drivers::arm_sdk ALIAS arm_sdk)

target_compile_features(arm_sdk PUBLIC cxx_std_17)

target_include_directories(arm_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(arm_sdk
    PUBLIC
        unitree_sdk2
        marinholab::sas::core
)

set_target_properties(arm_sdk PROPERTIES
    OUTPUT_NAME unitree_drivers_arm_sdk
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
