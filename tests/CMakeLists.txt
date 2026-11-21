find_package(GTest REQUIRED)
include(GoogleTest)

# One executable per library. The tests that use DDS share the same domain and
# topics, so they must not run in parallel (RESOURCE_LOCK).
foreach(library low_cmd low_state arm_sdk loco_client)
    add_executable(test_${library} main.cpp test_${library}.cpp)
    target_link_libraries(test_${library} PRIVATE
        ${library}
        unitree_sdk2
        GTest::gtest
    )
    gtest_discover_tests(test_${library}
        PROPERTIES RESOURCE_LOCK dds TIMEOUT 60
    )
endforeach()
