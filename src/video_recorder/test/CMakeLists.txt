find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)


# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()


ament_add_gmock(video_recorder_subscriber_test video_recorder_subscriber_test.cpp)
target_include_directories(video_recorder_subscriber_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(video_recorder_subscriber_test
    ${library_name}
    rclcpp::rclcpp
    image_transport::image_transport
)

