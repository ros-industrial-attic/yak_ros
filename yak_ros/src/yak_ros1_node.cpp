#include <ros/ros.h>

#include <yak/yak_server.h>
#include <yak/mc/marching_cubes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cv_bridge/cv_bridge.h>

#include <std_srvs/Trigger.h>
#include <yak_ros_msgs/UpdateKinFuParams.h>
#include <yak_ros/utils.h>

#include <yak_ros/online_fusion_server_ros1.h>

using namespace yak_ros;

static const std::double_t DEFAULT_MINIMUM_TRANSLATION = 0.00001;

OnlineFusionServer::OnlineFusionServer(ros::NodeHandle& nh,
                                       const kfusion::KinFuParams& params,
                                       const std::string& tsdf_frame)
  : fusion_(params, Eigen::Affine3f::Identity())
  , params_(params)
  , robot_tform_listener_(tf_buffer_)
  , tsdf_frame_to_camera_prev_(Eigen::Affine3d::Identity())
  , tsdf_frame_(tsdf_frame)
{
  // Subscribe to depth images published on the topic named by the depth_topic param. Set up callback to integrate
  // images when received.
  depth_image_raw_sub_ = nh.subscribe("input_depth_image", 1, &OnlineFusionServer::onReceivedDepthImg, this);

  // Advertise service for marching cubes meshing
  generate_mesh_service_ = nh.advertiseService("generate_mesh", &OnlineFusionServer::onGenerateMesh, this);

  // Advertise service to reset tsdf volume
  reset_tsdf_service_ = nh.advertiseService("reset_tsdf", &OnlineFusionServer::onReset, this);

  // Advertise service to update the params
  reset_tsdf_service_ = nh.advertiseService("update_params", &OnlineFusionServer::onUpdateParams, this);
}

void OnlineFusionServer::onReceivedDepthImg(const sensor_msgs::ImageConstPtr& image_in)
{
  // Get the camera pose in the world frame at the time when the depth image was generated.
  ROS_INFO_STREAM("Got depth image");
  geometry_msgs::TransformStamped transform_tsdf_frame_to_camera;
  try
  {
    transform_tsdf_frame_to_camera =
        tf_buffer_.lookupTransform(tsdf_frame_, image_in->header.frame_id, image_in->header.stamp);
  }
  catch (tf2::TransformException& ex)
  {
    // Abort integration if tf lookup failed
    ROS_WARN("%s", ex.what());
    return;
  }
  Eigen::Affine3d tsdf_frame_to_camera = tf2::transformToEigen(transform_tsdf_frame_to_camera);

  // Find how much the camera moved since the last depth image. If the magnitude of motion was below some threshold,
  // abort integration. This is to prevent noise from accumulating in the isosurface due to numerous observations from
  // the same pose.
  std::double_t motion_mag = (tsdf_frame_to_camera.inverse() * tsdf_frame_to_camera_prev_).translation().norm();
  ROS_INFO_STREAM(motion_mag);
  if (motion_mag < DEFAULT_MINIMUM_TRANSLATION)
  {
    ROS_INFO_STREAM("Camera motion below threshold");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_in, sensor_msgs::image_encodings::TYPE_16UC1);

  // Integrate the depth image into the TSDF volume
  if (!fusion_.fuse(cv_ptr->image, tsdf_frame_to_camera.cast<float>()))
    ROS_WARN_STREAM("Failed to fuse image");

  // If integration was successful, update the previous camera pose with the new camera pose
  tsdf_frame_to_camera_prev_ = tsdf_frame_to_camera;
  return;
}

bool OnlineFusionServer::onGenerateMesh(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  (void)req;
  yak::MarchingCubesParameters mc_params;
  mc_params.scale = static_cast<double>(params_.volume_resolution);
  pcl::PolygonMesh mesh = yak::marchingCubesCPU(fusion_.downloadTSDF(), mc_params);
  ROS_INFO_STREAM("Meshing done, saving ply");
  pcl::io::savePLYFileBinary("cubes.ply", mesh);
  ROS_INFO_STREAM("Saving done");
  res.success = true;
  return true;
}

bool OnlineFusionServer::onReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  (void)req;
  if (fusion_.reset())
  {
    ROS_INFO_STREAM("TSDF volume has been reset");
    res.success = true;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("TSDF volume has not been reset");
    res.success = false;
    return false;
  }
}

bool OnlineFusionServer::onUpdateParams(yak_ros_msgs::UpdateKinFuParamsRequest& req,
                                        yak_ros_msgs::UpdateKinFuParamsResponse& res)
{
  if (!yak_ros::updateParams(params_, req))
  {
    ROS_ERROR_STREAM("Update KinFuParams failed. TSDF volume has not been reset.");
    res.success = false;
    return false;
  }
  if (!fusion_.resetWithNewParams(params_))
  {
    ROS_ERROR_STREAM("Resetting Fusion Server Failed.");
    res.success = false;
    return false;
  }
  else
  {
    ROS_INFO_STREAM("TSDF volume has been reset and " << req.params_to_update.size()
                                                      << " parameters have been updated.");
    res.success = true;
    return true;
  }
}

/**
 * @brief main - Initialize the tsdf_node
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tsdf_node");
  ros::NodeHandle pnh("~");

  kfusion::KinFuParams default_params = kfusion::KinFuParams::default_params();
  default_params.use_pose_hints = true;  // use robot forward kinematics to find camera pose relative to TSDF volume
  default_params.use_icp = false;  // since we're using robot FK to get the camera pose, don't use ICP (TODO: yet!)
  pnh.param<bool>("update_via_sensor_motion", default_params.update_via_sensor_motion, false);

  // Get camera intrinsics from params
  XmlRpc::XmlRpcValue camera_matrix;
  pnh.getParam("camera_matrix", camera_matrix);
  default_params.intr.fx = static_cast<float>(static_cast<double>(camera_matrix[0]));
  default_params.intr.fy = static_cast<float>(static_cast<double>(camera_matrix[4]));
  default_params.intr.cx = static_cast<float>(static_cast<double>(camera_matrix[2]));
  default_params.intr.cy = static_cast<float>(static_cast<double>(camera_matrix[5]));

  ROS_INFO("Camera Intr Params: %f %f %f %f\n",
           static_cast<double>(default_params.intr.fx),
           static_cast<double>(default_params.intr.fy),
           static_cast<double>(default_params.intr.cx),
           static_cast<double>(default_params.intr.cy));

  // Set up TSDF parameters
  // TODO: Autocompute resolution from volume length/width/height in meters
  int volume_x, volume_y, volume_z;
  pnh.param<int>("volume_x", volume_x, 640);
  pnh.param<int>("volume_y", volume_y, 640);
  pnh.param<int>("volume_z", volume_z, 640);
  pnh.param<float>("volume_resolution", default_params.volume_resolution, 0.002f);
  default_params.volume_dims = cv::Vec3i(volume_x, volume_y, volume_z);

  default_params.volume_pose =
      Eigen::Affine3f::Identity();  // This is not settable via ROS. Change by moving TSDF frame
  pnh.param<float>(
      "tsdf_trunc_dist", default_params.tsdf_trunc_dist, default_params.volume_resolution * 5.0f);  // meters;
  pnh.param<int>("tsdf_max_weight", default_params.tsdf_max_weight, 50);                            // frames
  pnh.param<float>("raycast_step_factor", default_params.raycast_step_factor, 0.25);                // in voxel sizes
  pnh.param<float>("gradient_delta_factor", default_params.gradient_delta_factor, 0.25);            // in voxel sizes

  std::string tsdf_frame;
  pnh.param<std::string>("tsdf_frame", tsdf_frame, "tsdf_frame");

  // Set up the fusion server with the above parameters;
  OnlineFusionServer ofs(pnh, default_params, tsdf_frame);

  ros::spin();
  return 0;
}
