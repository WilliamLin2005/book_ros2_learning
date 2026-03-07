cmake_minimum_required(VERSION 3.8)
project(bt_vff)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(visualization_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(ZMQ REQUIRED)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    sensor_msgs
    geometry_msgs
    ament_index_cpp
    visualization_msgs
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(vff_bt_node SHARED src/bt_vff/vff_avoidance.cpp)
add_library(Back_bt_node SHARED src/bt_vff/Back.cpp)
add_library(Turn_bt_node SHARED src/bt_vff/Turn.cpp)
add_library(Stop_bt_node SHARED src/bt_vff/Stop.cpp)
add_library(ready_to_run_vff_bt_node SHARED src/bt_vff/ready_to_run_vff.cpp)

list(APPEND plugin_libs
  vff_bt_node
  Back_bt_node
  Turn_bt_node
  Stop_bt_node
  ready_to_run_vff_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_vff src/bt_vff_main.cpp)
ament_target_dependencies(bt_vff ${dependencies})
target_link_libraries(bt_vff ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  bt_vff
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include) 
ament_export_dependencies(${dependencies})

ament_package()
