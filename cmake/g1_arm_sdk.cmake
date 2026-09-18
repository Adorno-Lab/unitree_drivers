# ---------------------------------------------------------------------------
# Library: g1_arm_sdk (wraps DriverUnitreeG1ArmSDK)
# ---------------------------------------------------------------------------
# No dqrobotics/Eigen dependency: DriverUnitreeG1ArmSDK.h only pulls in <array>,
# <atomic>, <memory>. unitree_sdk2 is used only in the .cpp, but is kept PUBLIC
# here anyway since it's the shared vocabulary type of this whole package family
# (consistent with loco_client) -- feel free to make it PRIVATE if that's not a
# concern for your consumers. sas_core_pure is PRIVATE -- see the note in
# CMakeLists.txt on why that forces this target to be SHARED rather than STATIC.
add_library(g1_arm_sdk SHARED
    src/DriverUnitreeG1ArmSDK.cpp
)
add_library(unitree_drivers::g1_arm_sdk ALIAS g1_arm_sdk)

target_compile_features(g1_arm_sdk PUBLIC cxx_std_17)

target_include_directories(g1_arm_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(g1_arm_sdk
    PUBLIC
        unitree_sdk2
    PRIVATE
        sas_core_pure
)

set_target_properties(g1_arm_sdk PROPERTIES
    OUTPUT_NAME unitree_drivers_g1_arm_sdk
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
