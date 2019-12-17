/**
 * @file visualizer_ros1.j
 * @brief Publishes visualization messages for RVIZ
 *
 * @author Matthew Powelson
 * @date December 2, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef YAK_ROS_VISUALIZER_ROS1_H
#define YAK_ROS_VISUALIZER_ROS1_H

#include <ros/ros.h>
#include <ros/timer.h>

namespace yak_ros
{
/**
 * @brief Structure for containing the information to define a bounding box
 */
struct BoundingBox
{
  BoundingBox() : neg_x_(0), neg_y_(0), neg_z_(0), pos_x_(1), pos_y_(1), pos_z_(1), frame_("tsdf_frame") {}
  BoundingBox(double& neg_x,
              double& neg_y,
              double& neg_z,
              double& pos_x,
              double& pos_y,
              double& pos_z,
              std::string& frame)
    : neg_x_(neg_x), neg_y_(neg_y), neg_z_(neg_z), pos_x_(pos_x), pos_y_(pos_y), pos_z_(pos_z), frame_(frame)
  {
  }
  /** @brief Location of yz plane in negative direction */
  double neg_x_;
  /** @brief Location of xz plane in negative direction */
  double neg_y_;
  /** @brief Location of xy plane in negative direction */
  double neg_z_;
  /** @brief Location of yz plane in positive direction */
  double pos_x_;
  /** @brief Location of xz plane in positive direction */
  double pos_y_;
  /** @brief Location of xy plane in positive direction */
  double pos_z_;
  /** @brief Frame to which these dimensions are relative */
  std::string frame_;
};

/**
 * @brief This class visualizes useful information for yak_ros1_node. It publishes the tsdf volume as a bounding box as
 * well as the resultant mesh.
 *
 * Note that these markers are published every second by a ROS timer that is started when the feature is set and stopped
 * when the feature is cleared.
 */
class VisualizerRos1
{
public:
  /**
   * @brief Constructor
   * @param nh Nodehandle associated with the marker publisher
   * @param frame_id The TF frame associated with tsdf volume.
   */
  VisualizerRos1(const ros::NodeHandle& nh, const std::string& frame_id);

  /**
   * @brief setBoundingBox Sets the size of the bounding box and resets the publisher
   * @param neg_x Location of yz plane in negative direction
   * @param neg_y Location of xz plane in negative direction
   * @param neg_z Location of xy plane in negative direction
   * @param pos_x Location of yz plane in positive direction
   * @param pos_y Location of xz plane in positive direction
   * @param pos_z Location of xy plane in positive direction
   */
  void setBoundingBox(const double& neg_x,
                      const double& neg_y,
                      const double& neg_z,
                      const double& pos_x,
                      const double& pos_y,
                      const double& pos_z);
  /** @brief Clears the bounding box marker and stops the bounding box timer */
  void clearBoundingBox();
  /**
   * @brief Sets the mesh resource associated with the mesh marker and starts the mesh marker timer
   * @param mesh_resource The mesh resource for the mesh marker
   */
  void setMeshResource(const std::string& mesh_resource);
  /** @brief Clears the mesh marker and stops mesh marker timer */
  void clearMesh();
  /** @brief Frame with which the mesh markers are associated */
  std::string mesh_frame_id_;
  /** @brief Frame with which the bounding box markers are associated */
  std::string bbox_frame_id_;

private:
  /** @brief ROS timer callback to publish the bounding box */
  void publishBoundingBoxCallback(const ros::TimerEvent&);
  /** @brief ROS timer callback to publish the mesh */
  void publishMeshCallback(const ros::TimerEvent&);
  /** @brief Bounding box defining the TSDF Volume */
  BoundingBox bbox_;
  /** @brief Mesh resource defining the TSDF results */
  std::string mesh_resource_;
  /** @brief Timer used to publish the bounding box at regular intervals (1 second) */
  ros::Timer bbox_timer_;
  /** @brief Timer used to publish the mesh at regular intervals (1 second) */
  ros::Timer mesh_timer_;
  /** @brief Nodehandle associated with the marker publisher */
  ros::NodeHandle nh_;
  /** @brief Publisher for markers */
  ros::Publisher vis_publisher_;
};
}  // namespace yak_ros

#endif
