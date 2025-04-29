# cmake_minimum_required(VERSION 3.8)
# project(ros2_demo_project)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)  # 确保tf2_ros被找到
# find_package(geometry_msgs REQUIRED)  # geometry_msgs也需要确保
# find_package(visualization_msgs REQUIRED)

# # 添加可执行文件
# add_executable(thingy52_node src/thingy52_node.cpp)
# ament_target_dependencies(thingy52_node rclcpp std_msgs)

# add_executable(mmwave_radar_node src/mmwave_radar_node.cpp)
# ament_target_dependencies(mmwave_radar_node rclcpp sensor_msgs)

# add_executable(kinect_node src/kinect_node.cpp)
# ament_target_dependencies(kinect_node rclcpp std_msgs sensor_msgs)  # 添加sensor_msgs依赖

# # 添加 static_map_node 可执行文件
# add_executable(static_map_node src/static_map_node.cpp)
# ament_target_dependencies(static_map_node rclcpp tf2_ros geometry_msgs)
# # tf2_ros

# add_executable(node_monitor_node src/node_monitor_node.cpp)
# ament_target_dependencies(node_monitor_node rclcpp rcl_interfaces)

# # 构建房间模型节点
# add_executable(room_map_node src/room_map_node.cpp)
# ament_target_dependencies(room_map_node rclcpp visualization_msgs)

# # 安装可执行文件
# install(TARGETS
#   thingy52_node
#   mmwave_radar_node
#   kinect_node
#   static_map_node
#   room_map_node
#   node_monitor_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # 安装启动文件
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# # 安装配置文件
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# # 测试（可选）
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()



cmake_minimum_required(VERSION 3.8)
project(ros2_demo_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  # 确保tf2_ros被找到
find_package(geometry_msgs REQUIRED)  # geometry_msgs也需要确保
find_package(visualization_msgs REQUIRED)

# 添加 C++ 可执行文件
add_executable(thingy52_node src/thingy52_node.cpp)
ament_target_dependencies(thingy52_node rclcpp std_msgs)

add_executable(mmwave_radar_node src/mmwave_radar_node.cpp)
ament_target_dependencies(mmwave_radar_node rclcpp sensor_msgs)

add_executable(kinect_node src/kinect_node.cpp)
ament_target_dependencies(kinect_node rclcpp std_msgs sensor_msgs)  # 添加sensor_msgs依赖

# 添加 static_map_node 可执行文件
add_executable(static_map_node src/static_map_node.cpp)
ament_target_dependencies(static_map_node rclcpp tf2_ros geometry_msgs)

add_executable(node_monitor_node src/node_monitor_node.cpp)
ament_target_dependencies(node_monitor_node rclcpp rcl_interfaces)

# 构建房间模型节点
add_executable(room_map_node src/room_map_node.cpp)
ament_target_dependencies(room_map_node rclcpp visualization_msgs)

# 添加 Python API Server 可执行文件
install(PROGRAMS
  src/api_server.py  # 将 api_server.py 注册为可执行文件
  DESTINATION lib/${PROJECT_NAME}  # 指定安装路径
)

# 安装可执行文件
install(TARGETS
  thingy52_node
  mmwave_radar_node
  kinect_node
  static_map_node
  room_map_node
  node_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 GUI 文件夹
install(DIRECTORY gui
  DESTINATION share/${PROJECT_NAME}/  # GUI 文件夹的安装路径
)

# 测试（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
