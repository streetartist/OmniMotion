cmake_minimum_required(VERSION 3.14)

# BLDC FOC example
add_executable(bldc_foc_example bldc_foc/main.cpp)
target_link_libraries(bldc_foc_example PRIVATE omnimotion)

# Stepper S-curve example
add_executable(stepper_scurve_example stepper_scurve/main.cpp)
target_link_libraries(stepper_scurve_example PRIVATE omnimotion)

# Multi-axis synchronization example
add_executable(multi_axis_sync_example multi_axis_sync/main.cpp)
target_link_libraries(multi_axis_sync_example PRIVATE omnimotion)

# Trajectory tracking example
add_executable(trajectory_tracking_example trajectory_tracking/main.cpp)
target_link_libraries(trajectory_tracking_example PRIVATE omnimotion)

# Teach playback example
add_executable(teach_playback_example teach_playback/main.cpp)
target_link_libraries(teach_playback_example PRIVATE omnimotion)
