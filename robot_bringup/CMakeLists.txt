cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_library(NNG names nng)
find_library(MCX_CPP names mcx-cpp)
find_library(MCX_ROBOT_CONTROL names mcx-robot-control)

add_executable(robot_commander src/commander.cpp)


#target_link_libraries(robot_commander ${NNG} ${MCX_CPP} ${MCX_ROBOT_CONTROL} )
target_include_directories(robot_commander PUBLIC include)

install(TARGETS robot_commander
        DESTINATION lib
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES resource/)


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
