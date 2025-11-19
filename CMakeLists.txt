cmake_minimum_required(VERSION 3.8)
project(vosk_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Transcription.msg"
  "msg/GrammarConfig.msg"
  "action/STT.action"
  DEPENDENCIES std_msgs
)

# Install Python executable
install(PROGRAMS
  src/vosk_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vosk_node
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
