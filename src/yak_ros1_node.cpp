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

static const std::double_t DEFAULT_MINIMUM_TRANSLATION = 0.00001;

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
  OnlineFusionServer(ros::NodeHandle& nh, const kfusion::KinFuParams& params, const std::string& tsdf_frame)
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
    generate_mesh_service_ = nh.advertiseService("generate_mesh_service", &OnlineFusionServer::onGenerateMesh, this);
  }

private:
  /**
   * @brief onReceivedDepthImg - callback for integrating new depth images into the TSDF volume
   * @param image_in - pointer to new depth image
   */
  void onReceivedDepthImg(const sensor_msgs::ImageConstPtr& image_in)
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

  /**
   * @brief onGenerateMesh - Perform marching cubes meshing on the TSDF volume and save the result as a binary .ply
   * file.
   * @param req
   * @param res
   * @return
   */
  bool onGenerateMesh(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    yak::MarchingCubesParameters mc_params;
    mc_params.scale = params_.volume_resolution;
    pcl::PolygonMesh mesh = yak::marchingCubesCPU(fusion_.downloadTSDF(), mc_params);
    ROS_INFO_STREAM("Meshing done, saving ply");
    pcl::io::savePLYFileBinary("cubes.ply", mesh);
    ROS_INFO_STREAM("Saving done");
    res.success = true;
    return true;
  }

  yak::FusionServer fusion_;
  const kfusion::KinFuParams params_;

  /** @brief Subscriber that listens to incoming unrectofied depth images */
  ros::Subscriber depth_image_raw_sub_;
  /** @brief Buffer used to store locations of the camera (*/
  tf2_ros::Buffer tf_buffer_;
  /** @brief Listener used to look up tranforms for the location of the camera */
  tf2_ros::TransformListener robot_tform_listener_;
  /** @brief Service that is called to trigger marching cubes and saving the mesh */
  ros::ServiceServer generate_mesh_service_;
  /** @brief Service that resets the tsdf volue */
  ros::ServiceServer reset_tsdf_service_;
  /** @brief Used to track if the camera has moved. Only add image if it has */
  Eigen::Affine3d tsdf_frame_to_camera_prev_;
  /** @brief TF frame associated with the TSDF volume. */
  std::string tsdf_frame_;
};

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
  pnh.getParam("camera/camera_matrix/data", camera_matrix);
  default_params.intr.fx = static_cast<double>(camera_matrix[0]);
  default_params.intr.fy = static_cast<double>(camera_matrix[4]);
  default_params.intr.cx = static_cast<double>(camera_matrix[2]);
  default_params.intr.cy = static_cast<double>(camera_matrix[5]);

  ROS_INFO("Camera Intr Params: %f %f %f %f\n",
           default_params.intr.fx,
           default_params.intr.fy,
           default_params.intr.cx,
           default_params.intr.cy);

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
