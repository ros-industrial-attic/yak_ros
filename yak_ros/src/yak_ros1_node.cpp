#include <ros/ros.h>

#include <yak/yak_server.h>
#include <yak/mc/marching_cubes.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

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
  , visualizer_(nh, tsdf_frame)
{
  // Subscribe to depth images published on the topic named by the depth_topic param. Set up callback to integrate
  // images when received.
  depth_image_raw_sub_ = nh.subscribe("input_depth_image", 1, &OnlineFusionServer::onReceivedDepthImg, this);

  // Subscribe to point cloud
  point_cloud_sub_ = nh.subscribe("input_point_cloud", 1, &OnlineFusionServer::onReceivedPointCloud, this);

  // Advertise service for marching cubes meshing
  generate_mesh_service_ = nh.advertiseService("generate_mesh", &OnlineFusionServer::onGenerateMesh, this);

  // Advertise service to reset tsdf volume
  reset_tsdf_service_ = nh.advertiseService("reset_tsdf", &OnlineFusionServer::onReset, this);

  // Advertise service to update the params
  update_params_service_ = nh.advertiseService("update_params", &OnlineFusionServer::onUpdateParams, this);

  // Publish the TSDF Volume
  visualizer_.setBoundingBox(0,
                             0,
                             0,
                             params.volume_dims[0] * params.volume_resolution,
                             params.volume_dims[1] * params.volume_resolution,
                             params.volume_dims[2] * params.volume_resolution);
}

void OnlineFusionServer::onReceivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  if (cloud_in->height == 1)
  {
    ROS_ERROR("YAK_ROS only supports structured pointclouds. Cloud has not been integrated.");
    return;
  }

  // Consolidate important parameters
  const float centre_x = params_.intr.cx;
  const float centre_y = params_.intr.cy;
  const float focal_x = params_.intr.fx;
  const float focal_y = params_.intr.fy;
  const int image_height = cloud_in->height;
  const int image_width = cloud_in->width;

  // Convert to useful point cloud format
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

  // Convert to depth image
  cv::Mat cv_image = cv::Mat(image_height, image_width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
  int pixel_outside_bounds = 0;
  for (std::size_t i = 0; i < cloud->points.size(); i++)
  {
    if (cloud->points[i].z == cloud->points[i].z)
    {
      float z = cloud->points[i].z * 1000.0;
      float u = (cloud->points[i].x * 1000.0 * focal_x) / z;
      float v = (cloud->points[i].y * 1000.0 * focal_y) / z;
      int pixel_pos_x = (int)(u + centre_x);
      int pixel_pos_y = (int)(v + centre_y);

      if (pixel_pos_x < 0 || pixel_pos_y < 0 || pixel_pos_x > (image_width - 1) || pixel_pos_y > (image_height - 1))
      {
        // Count the amount of points which were projected out of depth image bounds.
        pixel_outside_bounds++;
        continue;
      }
      cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
    }
  }
  if (pixel_outside_bounds != 0)
  {
    // Print warning if projected point is outside of image bounds
    ROS_WARN_STREAM("Discarded " << pixel_outside_bounds << " points because they were out of bounds. "
                                 << "Your camera matrix might be set incorrectly.");
  }

  // Convert to message
  cv_image.convertTo(cv_image, CV_16UC1);
  sensor_msgs::ImagePtr output_image =
      cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, cv_image).toImageMsg();
  output_image->header = cloud_in->header;

  // Send to depth image callback
  onReceivedDepthImg(output_image);
}

void OnlineFusionServer::onReceivedDepthImg(const sensor_msgs::ImageConstPtr& image_in)
{
  // Get the camera pose in the world frame at the time when the depth image was generated.
  ROS_DEBUG("Got depth image");
  image_queue_.push(image_in);
  geometry_msgs::TransformStamped transform_tsdf_frame_to_camera;
  if (image_queue_.size() > 0)
  {
    try
    {
      sensor_msgs::ImageConstPtr next_image = image_queue_.front();
      transform_tsdf_frame_to_camera =
          tf_buffer_.lookupTransform(tsdf_frame_, next_image->header.frame_id, next_image->header.stamp);

      ROS_DEBUG("Found valid transform between %s and %s", tsdf_frame_.c_str(), next_image->header.frame_id.c_str());

      image_queue_.pop();

      Eigen::Affine3d tsdf_frame_to_camera = tf2::transformToEigen(transform_tsdf_frame_to_camera);

      // Find how much the camera moved since the last depth image. If the magnitude of motion was below some threshold,
      // abort integration. This is to prevent noise from accumulating in the isosurface due to numerous observations
      // from the same pose.
      std::double_t motion_mag = (tsdf_frame_to_camera.inverse() * tsdf_frame_to_camera_prev_).translation().norm();
      ROS_DEBUG("Frame %s moved %f m since last image", next_image->header.frame_id.c_str(), motion_mag);
      if (motion_mag < DEFAULT_MINIMUM_TRANSLATION)
      {
        ROS_DEBUG("Motion below threshold, image will not be fused");
        return;
      }

      ROS_DEBUG("Image encoding is %s", next_image->encoding.c_str());
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(next_image, next_image->encoding);

      // Integrate the depth image into the TSDF volume
      if (!fusion_.fuse(cv_ptr->image, tsdf_frame_to_camera.cast<float>()))
        ROS_ERROR("Failed to fuse image.");

      // If integration was successful, update the previous camera pose with the new camera pose
      tsdf_frame_to_camera_prev_ = tsdf_frame_to_camera;

      ROS_DEBUG("Successfully fused image");
    }
    catch (tf2::TransformException& ex)
    {
      // Abort integration if tf lookup failed
      ROS_WARN("TF lookupTransform error: %s", ex.what());
      // Remove image if TF lookup failed because it is in the unknown past
      if (std::string(ex.what()).find(std::string("past")) != std::string::npos)
        image_queue_.pop();
      return;
    }
  }
  return;
}

bool OnlineFusionServer::onGenerateMesh(yak_ros_msgs::GenerateMeshRequest& req, yak_ros_msgs::GenerateMeshResponse& res)
{
  // Run marching cubes to mesh TSDF
  ROS_INFO("Beginning marching cubes meshing");
  yak::MarchingCubesParameters mc_params;
  mc_params.scale = static_cast<double>(params_.volume_resolution);
  pcl::PolygonMesh mesh = yak::marchingCubesCPU(fusion_.downloadTSDF(), mc_params);
  mesh.header.frame_id = tsdf_frame_;
  pcl_conversions::toPCL(ros::Time::now(), mesh.header.stamp);

  // Convert the results to the correct frame
  if (req.results_frame == "")
    req.results_frame = tsdf_frame_;
  transformPolygonMesh(mesh, req.results_frame);

  // Save the results to the appropriate directory
  std::string filename = req.results_dir + "/results_mesh.ply";
  res.results_path = filename;
  ROS_INFO("Meshing done, saving to %s", filename.c_str());
  pcl::io::savePLYFileBinary(filename, mesh);

  // Publish mesh results
  visualizer_.mesh_frame_id_ = req.results_frame;
  visualizer_.setMeshResource(filename);

  ROS_DEBUG("Done saving output mesh");
  res.success = true;
  return true;
}

bool OnlineFusionServer::onReset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  (void)req;
  if (fusion_.reset())
  {
    visualizer_.clearMesh();
    ROS_INFO("TSDF volume has been reset");
    res.success = true;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to reset TSDF volume");
    res.success = false;
    return false;
  }
}

bool OnlineFusionServer::onUpdateParams(yak_ros_msgs::UpdateKinFuParamsRequest& req,
                                        yak_ros_msgs::UpdateKinFuParamsResponse& res)
{
  if (!yak_ros::updateParams(params_, req))
  {
    ROS_ERROR("Update KinFuParams failed. TSDF volume has not been reset.");
    res.success = false;
    return false;
  }
  ROS_DEBUG("Successfully updated kinfu params");
  if (!fusion_.resetWithNewParams(params_))
  {
    ROS_ERROR("Resetting Fusion Server Failed.");
    res.success = false;
    return false;
  }
  else
  {
    // Publish the TSDF Volume
    visualizer_.setBoundingBox(0,
                               0,
                               0,
                               params_.volume_dims[0] * params_.volume_resolution,
                               params_.volume_dims[1] * params_.volume_resolution,
                               params_.volume_dims[2] * params_.volume_resolution);
    ROS_INFO_STREAM("TSDF volume has been reset and " << req.params_to_update.size()
                                                      << " parameters have been updated.");
    res.success = true;
    return true;
  }
}

bool OnlineFusionServer::transformPolygonMesh(pcl::PolygonMesh& input_mesh, const std::string& target_frame)
{
  if (!tf_buffer_._frameExists(input_mesh.header.frame_id))
  {
    ROS_ERROR("Input mesh frame %s does not exist", input_mesh.header.frame_id.c_str());
    return false;
  }
  if (!tf_buffer_._frameExists(target_frame))
  {
    ROS_ERROR("Target frame %s does not exist", target_frame.c_str());
    return false;
  }

  // Get tranform from current mesh frame to target frame
  ros::Time stamp;
  pcl_conversions::fromPCL(input_mesh.header.stamp, stamp);
  auto tsdf_to_target_tf = tf_buffer_.lookupTransform(target_frame, input_mesh.header.frame_id, stamp);
  Eigen::Affine3d tsdf_to_target = tf2::transformToEigen(tsdf_to_target_tf);

  // Tranform the point cloud to the correct frame
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(input_mesh.cloud, cloud);
  pcl::transformPointCloud(cloud, cloud, tsdf_to_target);
  pcl::toPCLPointCloud2(cloud, input_mesh.cloud);
  input_mesh.header.frame_id = target_frame;
  return true;
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

  kfusion::KinFuParams kinfu_params = kfusion::KinFuParams::default_params();

  // Size of the input image
  pnh.param<int>("cols", kinfu_params.cols, 640);
  pnh.param<int>("rows", kinfu_params.rows, 480);

  // Get camera intrinsics from params
  XmlRpc::XmlRpcValue camera_matrix;
  pnh.getParam("camera_matrix", camera_matrix);
  kinfu_params.intr.fx = static_cast<float>(static_cast<double>(camera_matrix[0]));
  kinfu_params.intr.fy = static_cast<float>(static_cast<double>(camera_matrix[4]));
  kinfu_params.intr.cx = static_cast<float>(static_cast<double>(camera_matrix[2]));
  kinfu_params.intr.cy = static_cast<float>(static_cast<double>(camera_matrix[5]));
  ROS_INFO("Camera Intr Params: %f %f %f %f\n",
           static_cast<double>(kinfu_params.intr.fx),
           static_cast<double>(kinfu_params.intr.fy),
           static_cast<double>(kinfu_params.intr.cx),
           static_cast<double>(kinfu_params.intr.cy));

  // Volume Dimensions - Must be multiples of 32
  // TODO: Autocompute resolution from volume length/width/height in meters
  int volume_x, volume_y, volume_z;
  pnh.param<int>("volume_x", volume_x, 640);
  pnh.param<int>("volume_y", volume_y, 640);
  pnh.param<int>("volume_z", volume_z, 640);
  if (volume_x % 32 != 0 || volume_y % 32 != 0 || volume_z % 32 != 0)
  {
    ROS_ERROR("Failed to initialize yak_ros node: Number of voxels in each dimension must be multiples of 32.");
    return -1;
  }

  pnh.param<float>("volume_resolution", kinfu_params.volume_resolution, 0.002f);
  kinfu_params.volume_dims = cv::Vec3i(volume_x, volume_y, volume_z);
  ROS_INFO_STREAM("TSDF Volume Dimensions (Voxels): " << kinfu_params.volume_dims);
  ROS_INFO_STREAM("TSDF Volume Resolution (m): " << kinfu_params.volume_resolution);

  // This is not settable via ROS. Change by moving TSDF frame in TF
  kinfu_params.volume_pose = Eigen::Affine3f::Identity();
  std::string tsdf_frame;
  pnh.param<std::string>("tsdf_frame", tsdf_frame, "tsdf_frame");

  // Other parameters
  pnh.param<float>("bilateral_sigma_depth", kinfu_params.bilateral_sigma_depth, 0.04f);
  pnh.param<float>("bilateral_sigma_spatial", kinfu_params.bilateral_sigma_spatial, 4.5f);
  pnh.param<int>("bilateral_kernal_size", kinfu_params.bilateral_kernel_size, 7);

  // This is the internal tsdf min camera movement, not the one implemented in this node (See
  // DEFAULT_MINIMUM_TRANSLATION)
  pnh.param<float>("tsdf_min_camera_movement", kinfu_params.tsdf_min_camera_movement, 0.f);

  pnh.param<float>("tsdf_trunc_dist", kinfu_params.tsdf_trunc_dist, kinfu_params.volume_resolution * 5.0f);  // meters;
  pnh.param<int>("tsdf_max_weight", kinfu_params.tsdf_max_weight, 50);                                       // frames
  pnh.param<float>("raycast_step_factor", kinfu_params.raycast_step_factor, 0.25);      // in voxel sizes
  pnh.param<float>("gradient_delta_factor", kinfu_params.gradient_delta_factor, 0.25);  // in voxel sizes
  float light_pose_x, light_pose_y, light_pose_z;
  pnh.param<float>("light_pose_x", light_pose_x, 0.f);
  pnh.param<float>("light_pose_y", light_pose_y, 0.f);
  pnh.param<float>("light_pose_z", light_pose_z, 0.f);
  kinfu_params.light_pose = cv::Vec3f(light_pose_x, light_pose_y, light_pose_z);

  kinfu_params.use_pose_hints = true;  // use robot forward kinematics to find camera pose relative to TSDF volume
  kinfu_params.use_icp = false;        // since we're using robot FK to get the camera pose, don't use ICP (TODO: yet!)
  pnh.param<bool>("update_via_sensor_motion", kinfu_params.update_via_sensor_motion, false);

  // Set up the fusion server with the above parameters;
  OnlineFusionServer ofs(pnh, kinfu_params, tsdf_frame);

  ros::spin();
  return 0;
}
