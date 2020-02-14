cmake_minimum_required(VERSION 3.5.0)
project(yak_ros_msgs)

# ROS1
if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
  )
  
  
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  
  # Generate messages in the 'msg' folder
  add_message_files(
    FILES
    KinFuParams.msg
  )
  
  # Generate services in the 'srv' folder
   add_service_files(
     FILES
     GenerateMesh.srv
     UpdateKinFuParams.srv
   )
  
  ## Generate actions in the 'action' folder
  # add_action_files(
  #   FILES
  #   Action1.action
  #   Action2.action
  # )
  
  # Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
      geometry_msgs
      std_msgs
  )
  
  
  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES yak_ros_msgs
    CATKIN_DEPENDS
      geometry_msgs
      message_runtime
      std_msgs
  #  DEPENDS system_lib
  )
  
  ###########
  ## Build ##
  ###########
  
  ## Specify additional locations of header files
  ## Your package locations should be listed before other locations
  include_directories(
  # include
    ${catkin_INCLUDE_DIRS}
  )

# ROS2
else()
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)  
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(yak_ros_msgs
    "msg/KinFuParams.msg"
    "srv/GenerateMesh.srv"
    "srv/UpdateKinFuParams.srv"
    DEPENDENCIES 
    builtin_interfaces
    geometry_msgs
    std_msgs
  )

  ament_package()

endif()
