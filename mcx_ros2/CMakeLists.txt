cmake_minimum_required(VERSION 3.8)
project(mcx_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_library(NNG NAMES nng)
# find_library(MCX_CPP NAMES mcx-cpp)
find_library(MCX_CPP NAMES mcx-cpp)
# ament_target_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(test_node src/test.cpp)

ament_target_dependencies(test_node rclcpp std_msgs ament_index_cpp)
target_link_libraries(test_node ${MCX_CPP} ${NNG})
#target_include_directories(test_node PUBLIC helper.h)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES src/mcx.cert.pem
  DESTINATION share/${PROJECT_NAME}/resource
)

install(TARGETS
  test_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
