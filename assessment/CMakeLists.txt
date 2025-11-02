cmake_minimum_required(VERSION 3.8)
project(assessment)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(assessment_interfaces REQUIRED)
find_package(image_transport REQUIRED)

# ---- C++ node ----
add_executable(visual_sensor src/visual_sensor.cpp)
ament_target_dependencies(
  visual_sensor 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV
  assessment_interfaces 
  image_transport
)
install(TARGETS visual_sensor DESTINATION lib/${PROJECT_NAME})

# ---- Python installation
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  assessment/barrel_manager.py
  assessment/dynamic_mask.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Copy folders to install space ----
# Copy launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Copy config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Copy models files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Copy params files
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# Copy rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Copy world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
