# ---------------------------------------------------------------------------
# Library: videohub (wraps DriverUnitreeVideohub)
# ---------------------------------------------------------------------------
# Returns the JPEG bytes of videohub without decoding them, so it does not depend
# on OpenCV, Eigen, or dqrobotics. unitree_sdk2 stays PRIVATE: the public header
# only exposes std types and sas::ShutdownSignaler.

add_library(videohub SHARED
    src/DriverUnitreeVideohub.cpp
)
add_library(unitree_drivers::videohub ALIAS videohub)

target_compile_features(videohub PUBLIC cxx_std_17)

target_include_directories(videohub PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(videohub
    PUBLIC
        marinholab::sas::core
    PRIVATE
        unitree_sdk2
)

set_target_properties(videohub PROPERTIES
    OUTPUT_NAME unitree_drivers_videohub
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
