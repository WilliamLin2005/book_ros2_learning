cmake_minimum_required(VERSION 3.8)
project(tf2_learn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/tf2_learn/obstacle_detector.cpp
  src/tf2_learn/obstacle_monitor.cpp
  src/tf2_learn/obstacle_detector_improved.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(tf2_main src/tf2_main.cpp)
ament_target_dependencies(tf2_main ${dependencies})
target_link_libraries(tf2_main ${PROJECT_NAME})

add_executable(tf2_main_imp src/tf2_main_improved.cpp)
ament_target_dependencies(tf2_main_imp ${dependencies})
target_link_libraries(tf2_main_imp ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  tf2_main
  tf2_main_imp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  ) 
  
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
