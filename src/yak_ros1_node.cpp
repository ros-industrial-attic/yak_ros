#include <ros/ros.h>

#include <yak/yak_server.h>
#include <yak/mc/marching_cubes.h>

#include <gl_depth_sim/interfaces/opencv_interface.h>

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
 * @brief The OnlineFusionServer class. Integrate depth images into a TSDF volume. When requested, mesh the volume using marching cubes.
 * Note that this will work using both simulated and real robots and depth cameras.
 */
class OnlineFusionServer
{
public:
  /**
   * @brief OnlineFusionServer constructor
   * @param nh - ROS node handle
   * @param params - KinFu parameters such as TSDF volume size, resolution, etc.
   * @param world_to_volume - Transform from world frame to volume origin frame.
   */
  OnlineFusionServer(ros::NodeHandle &nh, const kfusion::KinFuParams& params, const Eigen::Affine3f& world_to_volume) :
    fusion_(params, world_to_volume),
    params_(params),
    robot_tform_listener_(tf_buffer_),
    world_to_camera_prev_(Eigen::Affine3d::Identity())
  {
    // Subscribe to depth images published on the topic named by the depth_topic param. Set up callback to integrate images when received.
    std::string depth_topic;
    nh.getParam("depth_topic", depth_topic);
    point_cloud_sub_ = nh.subscribe(depth_topic, 1, &OnlineFusionServer::onReceivedDepthImg, this);

    // Advertise service for marching cubes meshing
    generate_mesh_service_ = nh.advertiseService("generate_mesh_service", &OnlineFusionServer::onGenerateMesh, this);

    // Advertise service for clearing the contents of the voxel volume
    reset_volume_service_ = nh.advertiseService("reset_volume_service", &OnlineFusionServer::onResetVolume, this);

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
    geometry_msgs::TransformStamped transform_world_to_camera;
    try
    {
      transform_world_to_camera = tf_buffer_.lookupTransform("base_link", image_in->header.frame_id, image_in->header.stamp);
    }
    catch(tf2::TransformException &ex)
    {
      // Abort integration if tf lookup failed
      ROS_WARN("%s", ex.what());
      return;
    }
    Eigen::Affine3d world_to_camera = tf2::transformToEigen(transform_world_to_camera);

    // Find how much the camera moved since the last depth image. If the magnitude of motion was below some threshold, abort integration.
    // This is to prevent noise from accumulating in the isosurface due to numerous observations from the same pose.
    std::double_t motion_mag = (world_to_camera.inverse() * world_to_camera_prev_).translation().norm();
    ROS_INFO_STREAM(motion_mag);
    if(motion_mag < DEFAULT_MINIMUM_TRANSLATION)
    {
      ROS_INFO_STREAM("Camera motion below threshold");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_in, sensor_msgs::image_encodings::TYPE_16UC1);

    // Integrate the depth image into the TSDF volume
    if (!fusion_.fuse(cv_ptr->image, world_to_camera.cast<float>()))
      ROS_WARN_STREAM("Failed to fuse image");

    // If integration was successful, update the previous camera pose with the new camera pose
    world_to_camera_prev_ = world_to_camera;
    return;
  }

  /**
   * @brief onGenerateMesh - Perform marching cubes meshing on the TSDF volume and save the result as a binary .ply file.
   * @param req
   * @param res
   * @return
   */
  bool onGenerateMesh(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
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

  bool onResetVolume(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
  {
    ROS_INFO_STREAM("Reseting volume");
    res.success = fusion_.reset();;
    return true;
  }


  ros::Subscriber point_cloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener robot_tform_listener_;
  ros::ServiceServer generate_mesh_service_;
  ros::ServiceServer reset_volume_service_;
  yak::FusionServer fusion_;
  const kfusion::KinFuParams params_;
  Eigen::Affine3d world_to_camera_prev_;
};

/**
 * @brief main - Initialize the tsdf_node
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tsdf_node");
  ros::NodeHandle nh("tsdf_node");

  kfusion::KinFuParams default_params = kfusion::KinFuParams::default_params();
  default_params.use_pose_hints = true; // use robot forward kinematics to find camera pose relative to TSDF volume
  default_params.use_icp = false; // since we're using robot FK to get the camera pose, don't use ICP (TODO: yet!)
  default_params.update_via_sensor_motion = false;

  // Get camera intrinsics from params
  XmlRpc::XmlRpcValue camera_matrix;
  nh.getParam("camera/camera_matrix/data", camera_matrix);
  default_params.intr.fx = static_cast<double>(camera_matrix[0]);
  default_params.intr.fy = static_cast<double>(camera_matrix[4]);
  default_params.intr.cx = static_cast<double>(camera_matrix[2]);
  default_params.intr.cy = static_cast<double>(camera_matrix[5]);

  ROS_INFO("Camera Intr Params: %f %f %f %f\n", default_params.intr.fx, default_params.intr.fy, default_params.intr.cx, default_params.intr.cy);


  // TODO: Don't hardcode TSDF volume origin pose
  Eigen::Affine3f world_to_volume (Eigen::Affine3f::Identity());
  world_to_volume.translation() += Eigen::Vector3f(0.0f, -1.5f, -2.0f);

  // Set up TSDF parameters
  // TODO: Autocompute resolution from volume length/width/height in meters
  default_params.volume_dims = cv::Vec3i(640, 640, 640);
  default_params.volume_resolution = 0.005;
  default_params.volume_pose = Eigen::Affine3f::Identity(); // This gets overwritten when Yak is initialized
  default_params.tsdf_trunc_dist = default_params.volume_resolution * 5.0f; //meters;
  default_params.tsdf_max_weight = 50;   //frames
  default_params.raycast_step_factor = 0.25;  //in voxel sizes
  default_params.gradient_delta_factor = 0.25; //in voxel sizes

  // Set up the fusion server with the above parameters;
  OnlineFusionServer ofs(nh, default_params, world_to_volume);

  // Do TSDF-type things for the lifetime of the node
  ros::spin();
  return 0;
}
