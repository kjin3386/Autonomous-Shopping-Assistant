cmake_minimum_required(VERSION 3.8)
project(front_tracking_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 메시지 관련 의존성
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 메시지 파일 목록
set(msg_files
  "msg/PersonTrack.msg"
  "msg/PersonTrackArray.msg"
  "msg/PersonTrackPath.msg"
)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()

