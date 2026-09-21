# ---------------------------------------------------------------------------
# Library: loco_client (wraps DriverUnitreeLocoClient)
# ---------------------------------------------------------------------------
# PUBLIC deps: unitree_sdk2 and dqrobotics/Eigen appear in the installed public
# header (DriverUnitreeLocoClient.h includes <dqrobotics/DQ.h>), so consumers
# need them too.

add_library(loco_client SHARED
    src/DriverUnitreeLocoClient.cpp
)
add_library(unitree_drivers::loco_client ALIAS loco_client)

target_compile_features(loco_client PUBLIC cxx_std_17)

target_include_directories(loco_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(loco_client
    PUBLIC
        unitree_sdk2
        Eigen3::Eigen
        ${DQROBOTICS_LIBRARY}
        marinholab::sas::core
)

set_target_properties(loco_client PROPERTIES
    OUTPUT_NAME unitree_drivers_loco_client
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)
