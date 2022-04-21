cmake_minimum_required(VERSION 3.1)
project(landmarks_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Landmark_Msg.msg
  Landmarks_Msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)


