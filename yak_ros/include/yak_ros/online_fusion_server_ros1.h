/**
 * @file utils.h
 * @brief Utilities for converting formats in yak_ros
 *
 * @author Joe Schornak
 * @author Matthew Powelson
 * @date October 28, 2019
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
#ifndef YAK_ROS_ONLINE_FUSION_SERVER_H
#define YAK_ROS_ONLINE_FUSION_SERVER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/PolygonMesh.h>

#include <yak/yak_server.h>

#include <std_srvs/Trigger.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <yak_ros_msgs/GenerateMesh.h>
#include <yak_ros_msgs/UpdateKinFuParams.h>
#include <yak_ros/visualizer_ros1.h>

#include <queue>

namespace yak_ros
{
/**
 * @brief The OnlineFusionServer class. Integrate depth images into a TSDF volume. When requested, mesh the volume using
 * marching cubes. Note that this will work using both simulated and real robots and depth cameras.
 */
class OnlineFusionServer
{
public:
  /**
   * @brief OnlineFusionServer constructor
   * @param nh - ROS node handle
   * @param params - KinFu parameters such as TSDF volume size, resolution, etc.
   * @param tsdf_frame - Transform from world frame to volume origin frame.
   */
  OnlineFusionServer(ros::NodeHandle& nh, const kfusion::KinFuParams& params, const std::string& tsdf_frame);

private:
  /**
   * @brief onReceivePointCloud - callback for integrating new structured pointcloud into the TSDF volume
   * @param cloud_in - pointer to new point cloud
   */
  void onReceivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

  /**
   * @brief onReceivedDepthImg - callback for integrating new depth images into the TSDF volume
   * @param image_in - pointer to new depth image
   */
  void onReceivedDepthImg(const sensor_msgs::ImageConstPtr& image_in);

  /**
   * @brief onGenerateMesh - Perform marching cubes meshing on the TSDF volume and save the result as a binary .ply
   * file.
   * @param req Service request argument
   * @param req Service response argument
   * @return Returns true if successful
   */
  bool onGenerateMesh(yak_ros_msgs::GenerateMeshRequest& req, yak_ros_msgs::GenerateMeshResponse& res);

  /**
   * @brief Resets the tsdf volume
   * @param req Service request argument
   * @param res Service response argument
   * @return Returns true if successful
   */
  bool onReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  /**
   * @brief Updates the KinFu params and also resets the TSDF volume
   * @param req Service request argument
   * @param res Service response argument
   * @return Returns true if successful
   */
  bool onUpdateParams(yak_ros_msgs::UpdateKinFuParamsRequest& req, yak_ros_msgs::UpdateKinFuParamsResponse& res);

  /**
   * @brief Transforms a pcl::PolygonMesh into a new frame
   * @param input_mesh Mesh to be converted. Header must contain the current frame
   * @param target_frame Desired frame
   * @return Returns true if successful
   */
  bool transformPolygonMesh(pcl::PolygonMesh& input_mesh, const std::string& target_frame);

  /** @brief The fusion server that does the TSDF integration */
  yak::FusionServer fusion_;
  /** @brief The parameters that govern the TSDF integration */
  kfusion::KinFuParams params_;

  /** @brief Subscriber that listens to incoming unrectofied depth images */
  ros::Subscriber depth_image_raw_sub_;
  /** @brief Subscriber that listens to incoming structured point clouds.

  Note: Using depth images is prefered. This subscriber callback simply converts the point cloud to a depth image and
  calls the depth subscriber callback. If both are available, this is unnecessary overhead*/
  ros::Subscriber point_cloud_sub_;
  /** @brief Buffer used to store locations of the camera (*/
  tf2_ros::Buffer tf_buffer_;
  /** @brief Listener used to look up tranforms for the location of the camera */
  tf2_ros::TransformListener robot_tform_listener_;
  /** @brief Service that is called to trigger marching cubes and saving the mesh */
  ros::ServiceServer generate_mesh_service_;
  /** @brief Service that resets the tsdf volue */
  ros::ServiceServer reset_tsdf_service_;
  /** @brief Service that updates the KinFuParams and resets the TSDF volume */
  ros::ServiceServer update_params_service_;
  /** @brief Used to track if the camera has moved. Only add image if it has */
  Eigen::Affine3d tsdf_frame_to_camera_prev_;
  /** @brief TF frame associated with the TSDF volume. */
  std::string tsdf_frame_;
  /** @brief Queue that stores images until their transforms are available. This allows for the mismatch in timing
   * between images being published and TFs */
  std::queue<sensor_msgs::ImageConstPtr> image_queue_;
  /** @brief Used to visualize results in rviz */
  VisualizerRos1 visualizer_;
};
}  // namespace yak_ros

#endif
