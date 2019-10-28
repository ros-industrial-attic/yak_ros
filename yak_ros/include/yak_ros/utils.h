/**
 * @file utils.h
 * @brief Utilities for converting formats in yak_ros
 *
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
#ifndef YAK_ROS_UTILS_H
#define YAK_ROS_UTILS_H
#include <yak/kfusion/kinfu.hpp>
#include <yak_ros_msgs/UpdateKinFuParamsRequest.h>

namespace yak_ros
{
/**
 * @brief Update KinFuParams based on a given ROS service request.
 * @param params The params to be updated
 * @param request The service request. Note that there is a list of parameters to be updated since usually you only want
 * to change one or two of them
 * @return Returns true if successful.
 */
bool updateParams(kfusion::KinFuParams& params, yak_ros_msgs::UpdateKinFuParamsRequest& request)
{
  for (auto& param : request.params_to_update)
  {
    if (param == request.COLS)
    {
      params.cols = request.kinfu_params.cols;
    }
    else if (param == request.ROWS)
    {
      params.rows = request.kinfu_params.rows;
    }
    else if (param == request.INTR)
    {
      std::vector<float> intr = request.kinfu_params.intr;
      if (intr.size() != 4)
        return false;
      params.intr = kfusion::Intr(intr[0], intr[1], intr[2], intr[3]);
    }
    else if (param == request.VOLUME_DIMS)
    {
      std::vector<int> volume_dims = request.kinfu_params.volume_dims;
      if (volume_dims.size() != 3)
        return false;
      params.volume_dims = cv::Vec3i(volume_dims[0], volume_dims[1], volume_dims[2]);
    }
    else if (param == request.VOLUME_RESOLUTION)
    {
      params.volume_resolution = request.kinfu_params.volume_resolution;
    }
    else if (param == request.VOLUME_POSE)
    {
      // I think it is a bad idea to let people change this.
      return false;
    }
    else if (param == request.BILATERAL_SIGMA_DEPTH)
    {
      params.bilateral_sigma_depth = request.kinfu_params.bilateral_sigma_depth;
    }
    else if (param == request.BILATERAL_SIGMA_SPATIAL)
    {
      params.bilateral_sigma_spatial = request.kinfu_params.bilateral_sigma_spatial;
    }
    else if (param == request.BILATERAL_KERNAL_SIZE)
    {
      params.bilateral_kernel_size = request.kinfu_params.bilateral_kernel_size;
    }
    else if (param == request.ICP_TRUNCATE_DEPTH_DIST)
    {
      params.icp_truncate_depth_dist = request.kinfu_params.icp_truncate_depth_dist;
    }
    else if (param == request.ICP_DIST_THRES)
    {
      params.icp_dist_thres = request.kinfu_params.icp_dist_thres;
    }
    else if (param == request.ICP_ANGLE_THRES)
    {
      params.icp_angle_thres = request.kinfu_params.icp_angle_thres;
    }
    else if (param == request.ICP_ITER_NUM)
    {
      params.icp_iter_num = request.kinfu_params.icp_iter_num;
    }
    else if (param == request.TSDF_MIN_CAMERA_MOVEMENT)
    {
      params.tsdf_min_camera_movement = request.kinfu_params.tsdf_min_camera_movement;
    }
    else if (param == request.TSDF_TRUNC_DIST)
    {
      params.tsdf_trunc_dist = request.kinfu_params.tsdf_trunc_dist;
    }
    else if (param == request.TSDF_MAX_WEIGHT)
    {
      params.tsdf_max_weight = request.kinfu_params.tsdf_max_weight;
    }
    else if (param == request.RAYCAST_STEP_FACTOR)
    {
      params.raycast_step_factor = request.kinfu_params.raycast_step_factor;
    }
    else if (param == request.GRADIENT_DELTA_FACTOR)
    {
      params.gradient_delta_factor = request.kinfu_params.gradient_delta_factor;
    }
    else if (param == request.LIGHT_POSE)
    {
      std::vector<float> lp = request.kinfu_params.light_pose;
      if (lp.size() != 3)
        return false;
      params.light_pose = cv::Vec3f(lp[0], lp[1], lp[2]);
    }
    else if (param == request.USE_POSE_HINTS)
    {
      params.use_pose_hints = request.kinfu_params.use_pose_hints;
    }
    else if (param == request.USE_ICP)
    {
      params.use_icp = request.kinfu_params.use_icp;
    }
    else if (param == request.UPDATE_VIA_SENSOR_MOTION)
    {
      params.update_via_sensor_motion = request.kinfu_params.update_via_sensor_motion;
    }
  }

  return true;
}

}  // namespace yak_ros

#endif
