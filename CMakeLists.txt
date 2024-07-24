cmake_minimum_required(VERSION 3.8)
project(ros_zed_point_cloud_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ZED 4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(zed_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

add_executable(zed_filter src/zed_filter.cpp)
ament_target_dependencies(zed_filter rclcpp std_msgs sensor_msgs pcl_conversions pcl_ros)

add_executable(zed_segmentation src/zed_segmentation.cpp)
ament_target_dependencies(zed_segmentation rclcpp std_msgs sensor_msgs pcl_conversions pcl_ros)

add_executable(zed_cluster src/zed_cluster.cpp)
ament_target_dependencies(zed_cluster rclcpp std_msgs sensor_msgs pcl_conversions pcl_ros)

add_executable(zed_od src/zed_od.cpp)
ament_target_dependencies(zed_od rclcpp std_msgs std_srvs sensor_msgs pcl_conversions pcl_ros zed_components ZED OpenCV cv_bridge)

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

# Nodes to run
install(TARGETS
  zed_filter
  zed_segmentation
  zed_cluster
  zed_od
  DESTINATION lib/${PROJECT_NAME}
)

# Launch File path
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
