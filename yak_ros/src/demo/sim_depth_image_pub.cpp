#include <gl_depth_sim/sim_depth_camera.h>
#include <gl_depth_sim/mesh_loader.h>
#include <gl_depth_sim/interfaces/opencv_interface.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <chrono>

const static std::string DEFAULT_IMAGE_TOPIC = "image";

static Eigen::Isometry3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Isometry3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_simulator_node");
  ros::NodeHandle nh, pnh("~");

  // Setup ROS interfaces
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise(DEFAULT_IMAGE_TOPIC, 1);

  tf2_ros::TransformBroadcaster broadcaster;

  // Load ROS parameters
  std::string mesh_path;
  if (!pnh.getParam("mesh", mesh_path))
  {
    ROS_ERROR_STREAM("User must set the 'mesh' private parameter");
    return 1;
  }

  std::string base_frame = pnh.param<std::string>("base_frame", "world");
  std::string camera_frame = pnh.param<std::string>("camera_frame", "camera");

  double orbit_speed = pnh.param<double>("orbit_speed", 1.0);
  double framerate = pnh.param<double>("framerate", 30.0);

  double radius = pnh.param<double>("radius", 0.5);
  double z = pnh.param<double>("z", 0.5);

  double focal_length = pnh.param<double>("focal_length", 550.0);
  int width = pnh.param<int>("width", 640);
  int height = pnh.param<int>("height", 480);

  auto mesh_ptr = gl_depth_sim::loadMesh(mesh_path);

  if (!mesh_ptr)
  {
    ROS_ERROR_STREAM("Unable to load mesh from path: " << mesh_path);
    return 1;
  }

  gl_depth_sim::CameraProperties props;
  props.width = width;
  props.height = height;
  props.fx = focal_length;
  props.fy = focal_length;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25;
  props.z_far = 10.0f;

  // Create the simulation
  gl_depth_sim::SimDepthCamera sim(props);
  sim.add(*mesh_ptr, Eigen::Isometry3d::Identity());

  // State for FPS monitoring
  long frame_counter = 0;
  // In the main (rendering) thread, begin orbiting...
  const auto start = std::chrono::steady_clock::now();

  ros::Rate rate(framerate);

  while (ros::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    Eigen::Vector3d camera_pos(radius * cos(dt * orbit_speed), radius * sin(dt * orbit_speed), z);

    Eigen::Vector3d look_at(0, 0, 0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0, 0, 1));

    const auto depth_img = sim.render(pose);

    frame_counter++;

    if (frame_counter % 100 == 0)
    {
      std::cout << "FPS: " << frame_counter / dt << "\n";
    }

    cv::Mat cv_img;
    gl_depth_sim::toCvImage16u(depth_img, cv_img);
    cv_bridge::CvImage image;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = camera_frame;
    image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    image.image = cv_img;
    image_pub.publish(image.toImageMsg());

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.transform.translation.x = pose.translation().x();
    transform_stamped.transform.translation.y = pose.translation().y();
    transform_stamped.transform.translation.z = pose.translation().z();
    transform_stamped.transform.rotation = tf2::toMsg(Eigen::Quaternion<double>(pose.linear()));

    transform_stamped.header.stamp = image.header.stamp;
    transform_stamped.header.frame_id = base_frame;
    transform_stamped.child_frame_id = camera_frame;
    broadcaster.sendTransform(transform_stamped);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
