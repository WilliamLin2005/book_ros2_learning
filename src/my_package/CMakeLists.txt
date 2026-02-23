cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# declare ament package
set(dependencies
  rclcpp
  std_msgs
)

# build
add_executable(simple src/simple.cpp)
ament_target_dependencies(simple ${dependencies})
add_executable(logger src/logger.cpp)
ament_target_dependencies(logger ${dependencies})
add_executable(loggerclass src/loggerclass.cpp)
ament_target_dependencies(loggerclass ${dependencies})
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher ${dependencies})
add_executable(publisherclass src/publisherclass.cpp)
ament_target_dependencies(publisherclass ${dependencies})
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber ${dependencies})
add_executable(subscriberclass src/subscriberclass.cpp)
ament_target_dependencies(subscriberclass ${dependencies})
add_executable(parameters src/parameters.cpp)
ament_target_dependencies(parameters ${dependencies})
add_executable(executor src/executor.cpp)
ament_target_dependencies(executor ${dependencies})

# install
install(TARGETS
  simple
  logger
  loggerclass
  publisher
  publisherclass
  subscriber
  subscriberclass
  parameters
  executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

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

ament_export_dependencies(${dependencies})

ament_package()
