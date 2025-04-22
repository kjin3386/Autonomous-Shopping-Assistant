cmake_minimum_required(VERSION 3.10)
project(rear_tracking_node)

# 필수 패키지들
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# 메시지 파일 정의
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UserPolar.msg"
  DEPENDENCIES std_msgs
)

# 메시지를 Python에서 import할 수 있도록 설정
ament_python_install_package(${PROJECT_NAME})

# 의존성 처리
ament_export_dependencies(rosidl_default_runtime)
ament_package()

