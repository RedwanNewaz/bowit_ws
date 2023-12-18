cmake_minimum_required(VERSION 3.8)
project(bowit)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(limbo REQUIRED)
find_package(FCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${limbo_INCLUDE_DIRS}
    ${fcl_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# bowit library 
set(LIBBOW_SRC_FILE 
  src/libbow/bowit_params.h
  src/libbow/bowit_robot.h
  src/libbow/bowit_ros_interface.h
  src/libbow/collision_checker.h
  src/libbow/dataloader.h
  src/libbow/hypermap.h
  src/libbow/motion_model.h
  src/libbow/rapidcsv.h
)

add_executable(bowit_node 
    src/bowit_node.cpp
    ${LIBBOW_SRC_FILE} 
    )

target_link_libraries(bowit_node -pthread ${fcl_LIBRARIES}  ${limbo_LIBRARIES})

target_include_directories(bowit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bowit_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bowit_node
  "rclcpp"
  "tf2"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
)

add_executable(bowit_viz 
src/viz_node.cpp 
src/viz_interface/bowit_viz.cpp
${LIBBOW_SRC_FILE} 
)
target_link_libraries(bowit_viz -pthread ${fcl_LIBRARIES})
ament_target_dependencies(
  bowit_viz
  "rclcpp"
  "tf2"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
)

install(TARGETS bowit_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS bowit_viz
  DESTINATION lib/${PROJECT_NAME})

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


install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME})
