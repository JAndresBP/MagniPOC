cmake_minimum_required(VERSION 3.8)
project(video_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

set(executable_name ${PROJECT_NAME})
set(library_name ${executable_name}_core)

add_library(
  ${library_name} SHARED
  src/video_recorder_subscriber.cpp
  src/video_recorder.cpp
  src/mjpeg_recorder.cpp
)
target_include_directories(
  ${library_name} PUBLIC
  include
)
ament_target_dependencies(
  ${library_name}
  rclcpp 
  std_msgs
  sensor_msgs
  image_transport
)

add_executable(
  ${executable_name} 
  src/main.cpp
)
target_link_libraries(
  ${executable_name}
  ${library_name}
)
ament_target_dependencies(
  ${executable_name}
  rclcpp
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${executable_name}
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_include_directories(
  include
)

ament_package()
