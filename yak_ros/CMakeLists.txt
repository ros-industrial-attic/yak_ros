cmake_minimum_required(VERSION 3.5.0)
project(yak_ros VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(yak REQUIRED)

find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    # Build as a Catkin package for ROS

    find_package(catkin REQUIRED COMPONENTS
         roscpp
         pcl_ros
         tf2
         tf2_ros
         tf2_eigen
         cv_bridge
         yak_ros_msgs)

     catkin_package(
       INCLUDE_DIRS
       LIBRARIES
         ${yak_LIBRARIES}
         ${sensor_msgs_LIBRARIES}
         ${tf2_LIBRARIES}
         ${tf2_ros_LIBRARIES}
         ${tf2_eigen_LIBRARIES}
         ${cv_bridge_LIBRARIES}
       CATKIN_DEPENDS
         cv_bridge
         geometry_msgs
         pcl_ros
         sensor_msgs
         std_msgs
         visualization_msgs
         yak_ros_msgs
       DEPENDS
         yak
         tf2_eigen
         tf2)

     add_library(${PROJECT_NAME}_visualizer_ros1 src/yak_ros1/visualizer_ros1.cpp)
     add_dependencies(${PROJECT_NAME}_visualizer_ros1 ${catkin_EXPORTED_TARGETS})
     target_link_libraries(${PROJECT_NAME}_visualizer_ros1 ${catkin_LIBRARIES})
     target_include_directories(${PROJECT_NAME}_visualizer_ros1 PUBLIC
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
     target_include_directories(${PROJECT_NAME}_visualizer_ros1 PUBLIC ${catkin_INCLUDE_DIRS})

     add_executable(${PROJECT_NAME}_node
         src/yak_ros1_node.cpp)
     target_link_libraries(${PROJECT_NAME}_node
         ${CATKIN_LIBRARIES}
         ${sensor_msgs_LIBRARIES}
         ${tf2_LIBRARIES}
         ${tf2_ros_LIBRARIES}
         ${tf2_eigen_LIBRARIES}
         ${cv_bridge_LIBRARIES}
         yak::yak
         yak::yak_frontend
         yak::yak_marching_cubes
         ${PROJECT_NAME}_visualizer_ros1)
     target_include_directories(${PROJECT_NAME}_node PUBLIC
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
     target_include_directories(${PROJECT_NAME}_node PUBLIC
       ${catkin_INCLUDE_DIRS}
       ${gl_depth_sim_INCLUDE_DIRS})



    if(DEFINED BUILD_DEMO)
        find_package(gl_depth_sim REQUIRED)
        find_package(image_transport REQUIRED)
        add_executable(${PROJECT_NAME}_image_simulator
          src/demo/sim_depth_image_pub_ros1.cpp)
        target_link_libraries(${PROJECT_NAME}_image_simulator
          ${CATKIN_LIBRARIES}
          ${sensor_msgs_LIBRARIES}
          ${cv_bridge_LIBRARIES}
          ${gl_depth_sim_LIBRARIES}
          ${image_transport_LIBRARIES}
          Eigen3::Eigen)
        target_include_directories(${PROJECT_NAME}_image_simulator PUBLIC
          ${catkin_INCLUDE_DIRS}
          ${gl_depth_sim_INCLUDE_DIRS}
          ${image_transport_INCLUDE_DIRS}
          ${sensor_msgs_INCLUDE_DIRS}
        )
    endif()


else()
    # Build as an Ament package for ROS2
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    add_executable(${PROJECT_NAME}_node
        src/yak_ros2_node.cpp)

    target_include_directories(${PROJECT_NAME}_node PUBLIC
      ${rclcpp_INCLUDE_DIRS})

    target_link_libraries(${PROJECT_NAME}_node
        ${rclcpp_LIBRARIES}
        yak::yak
        yak::yak_frontend
        yak::yak_marching_cubes
        ${sensor_msgs_LIBRARIES}
        ${tf2_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        ${tf2_eigen_LIBRARIES}
        ${cv_bridge_LIBRARIES})

    ament_target_dependencies(${PROJECT_NAME}_node
                              "rclcpp"
                              "yak"
                              "std_srvs"
                              "sensor_msgs"
                              "geometry_msgs"
                              "tf2"
                              "tf2_ros"
                              "tf2_eigen")

    ament_export_include_directories(${rclcpp_INCLUDE_DIRS}
                                     ${yak_INCLUDE_DIRS})

    ament_export_dependencies(rclcpp yak tf2_eigen tf2)

    ament_package()
endif()

if ($ENV{ROS_VERSION} VERSION_EQUAL "1")
    set(ROS_LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(ROS_SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
    set(ROS_LIB_DESTINATION lib/${PROJECT_NAME})
    set(ROS_SHARE_DESTINATION share/${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${ROS_LIB_DESTINATION})

install(DIRECTORY launch demo
        DESTINATION ${ROS_SHARE_DESTINATION})

if (DEFINED BUILD_DEMO)
    install(TARGETS ${PROJECT_NAME}_image_simulator
            RUNTIME DESTINATION ${ROS_LIB_DESTINATION})
endif()


