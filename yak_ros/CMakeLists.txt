cmake_minimum_required(VERSION 3.5.0)
project(yak_ros VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(yak REQUIRED)

find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
     roscpp
     eigen_conversions
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
     eigen_conversions
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

 add_library(${PROJECT_NAME}_visualizer src/visualizer.cpp)
 add_dependencies(${PROJECT_NAME}_visualizer ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_visualizer ${catkin_LIBRARIES})
 target_include_directories(${PROJECT_NAME}_visualizer PUBLIC
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
     "$<INSTALL_INTERFACE:include>")
 target_include_directories(${PROJECT_NAME}_visualizer PUBLIC ${catkin_INCLUDE_DIRS})

 add_executable(${PROJECT_NAME}_node
     src/yak_node.cpp)
 target_include_directories(${PROJECT_NAME}_node PUBLIC
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
     "$<INSTALL_INTERFACE:include>")
 target_include_directories(${PROJECT_NAME}_node PUBLIC
   ${catkin_INCLUDE_DIRS})
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
     ${PROJECT_NAME}_visualizer)

if(BUILD_DEMO)
    find_package(gl_depth_sim REQUIRED)
    find_package(image_transport REQUIRED)
    add_executable(${PROJECT_NAME}_image_simulator
      src/demo/sim_depth_image_pub.cpp)
    target_include_directories(${PROJECT_NAME}_image_simulator PUBLIC
      ${catkin_INCLUDE_DIRS}
      ${gl_depth_sim_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME}_image_simulator
      ${CATKIN_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      ${cv_bridge_LIBRARIES}
      ${gl_depth_sim_LIBRARIES}
      ${image_transport_LIBRARIES}
      ${tf2_ros_LIBRARIES}
      Eigen3::Eigen)
endif()

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_visualizer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (DEFINED BUILD_DEMO)
    install(TARGETS ${PROJECT_NAME}_image_simulator
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

install(DIRECTORY launch demo
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
