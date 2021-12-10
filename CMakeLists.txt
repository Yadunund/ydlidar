cmake_minimum_required(VERSION 3.5)

project(ydlidar)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

#add_subdirectory(sdk)

set(SDK_PATH "./sdk/")

FILE(GLOB_RECURSE SDK_SRC
  "${SDK_PATH}/src/impl/unix/*.cpp"
  "${SDK_PATH}/src/*.cpp"
  "${SDK_PATH}/src/*.c"
)

include_directories(
  GNUInstallDirs
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/sdk/include
  ${PROJECT_SOURCE_DIR}/sdk/src
)

add_executable(ydlidar_node src/ydlidar_node.cpp  ${SDK_SRC})
add_executable(ydlidar_lifecycle_node src/ydlidar_lifecycle_node.cpp ${SDK_SRC})
add_executable(ydlidar_client src/ydlidar_client.cpp)

ament_target_dependencies(ydlidar_node
   rclcpp
   sensor_msgs
)

ament_target_dependencies(ydlidar_lifecycle_node
   rclcpp
   rclcpp_lifecycle
   sensor_msgs
)

ament_target_dependencies(ydlidar_client
   rclcpp
   sensor_msgs
)

install(TARGETS ydlidar_node ydlidar_lifecycle_node ydlidar_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch startup sdk
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
