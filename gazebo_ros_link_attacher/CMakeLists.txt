cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_link_attacher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Attach.srv"
)

# Add includes
include_directories(include)

# Declare plugin library with a different target name
add_library(${PROJECT_NAME}_plugin SHARED
  src/gazebo_ros_link_attacher.cpp
)

# Link against ROS 2 libraries
ament_target_dependencies(${PROJECT_NAME}_plugin
  "rclcpp"
  "gazebo_ros"
)

# Link against generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin "${cpp_typesupport_target}")

# Install the plugin library
install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/demo.py
  scripts/demo_multiple.py
  scripts/spawn_models.py
  scripts/attach.py
  scripts/detach.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()