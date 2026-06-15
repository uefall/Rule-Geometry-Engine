include(FetchContent)

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(rge_tests
    spatial_iou_test.cpp
    side_of_line_test.cpp
    trajectory_update_test.cpp
    trajectory_relation_test.cpp
    cross_line_test.cpp
    roi_enter_leave_test.cpp
)

target_link_libraries(rge_tests PRIVATE rge GTest::gtest_main)
rge_set_target_options(rge_tests)

include(GoogleTest)
gtest_discover_tests(rge_tests)
