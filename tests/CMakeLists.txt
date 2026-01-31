cmake_minimum_required(VERSION 3.14)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Test executable
add_executable(omnimotion_tests
    test_pid_controller.cpp
    test_trapezoidal_profile.cpp
    test_scurve_profile.cpp
    test_filters.cpp
    test_motor_state.cpp
)

target_link_libraries(omnimotion_tests PRIVATE
    omnimotion
    GTest::gtest_main
)

gtest_discover_tests(omnimotion_tests)
