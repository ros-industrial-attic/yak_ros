#include <yak_ros/visualizer_ros1.h>
#include <visualization_msgs/Marker.h>
#include <functional>

using namespace yak_ros;

/**
 * @brief Makes a geometry_msgs::Point from the 3 values
 * @param x x value in output point
 * @param y y value in output point
 * @param z z value in output point
 * @return Returnts a geometry_msgs::Point constructed from the xyz values passed in
 */
static inline geometry_msgs::Point MakePoint(const double& x, const double& y, const double& z)
{
  geometry_msgs::Point pnt;
  pnt.x = x;
  pnt.y = y;
  pnt.z = z;
  return pnt;
}

VisualizerRos1::VisualizerRos1(const ros::NodeHandle& nh, const std::string& bbox_frame_id)
  : bbox_frame_id_(bbox_frame_id), nh_(nh)
{
  vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  mesh_timer_ =
      nh_.createTimer(ros::Duration(1), std::bind(&VisualizerRos1::publishMeshCallback, this, std::placeholders::_1));
  bbox_timer_ = nh_.createTimer(ros::Duration(1),
                                std::bind(&VisualizerRos1::publishBoundingBoxCallback, this, std::placeholders::_1));
  clearMesh();
  clearBoundingBox();
}

void VisualizerRos1::setBoundingBox(const double& neg_x,
                                    const double& neg_y,
                                    const double& neg_z,
                                    const double& pos_x,
                                    const double& pos_y,
                                    const double& pos_z)
{
  bbox_.neg_x_ = neg_x;
  bbox_.neg_y_ = neg_y;
  bbox_.neg_z_ = neg_z;
  bbox_.pos_x_ = pos_x;
  bbox_.pos_y_ = pos_y;
  bbox_.pos_z_ = pos_z;
  bbox_.frame_ = bbox_frame_id_;

  bbox_timer_.start();
}

void VisualizerRos1::clearBoundingBox()
{
  bbox_timer_.stop();
  // Send delete
  visualization_msgs::Marker marker;
  marker.header.frame_id = bbox_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "yak_ros";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::DELETE;

  vis_publisher_.publish(marker);
}

void VisualizerRos1::setMeshResource(const std::string& mesh_resource)
{
  mesh_resource_ = "file://" + mesh_resource;
  mesh_timer_.start();
}

void VisualizerRos1::clearMesh()
{
  mesh_timer_.stop();
  // Send delete
  visualization_msgs::Marker marker;
  marker.header.frame_id = mesh_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "yak_ros";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::DELETE;

  vis_publisher_.publish(marker);
}

void VisualizerRos1::publishBoundingBoxCallback(const ros::TimerEvent&)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = bbox_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "yak_ros";
  marker.id = 1;
  marker.scale.x = 0.005;  // Line width
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  //  marker.frame_locked = true;  // Should be enabled if the frame is moving

  // X Direction
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.neg_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.neg_y_, bbox_.neg_z_));

  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.pos_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.pos_y_, bbox_.neg_z_));

  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.neg_y_, bbox_.pos_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.neg_y_, bbox_.pos_z_));

  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.pos_y_, bbox_.pos_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.pos_y_, bbox_.pos_z_));

  // Y Direction
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.neg_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.pos_y_, bbox_.neg_z_));

  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.neg_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.pos_y_, bbox_.neg_z_));

  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.neg_y_, bbox_.pos_z_));
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.pos_y_, bbox_.pos_z_));

  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.neg_y_, bbox_.pos_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.pos_y_, bbox_.pos_z_));

  // Z Direction
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.neg_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.neg_y_, bbox_.pos_z_));

  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.neg_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.neg_y_, bbox_.pos_z_));

  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.pos_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.neg_x_, bbox_.pos_y_, bbox_.pos_z_));

  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.pos_y_, bbox_.neg_z_));
  marker.points.push_back(MakePoint(bbox_.pos_x_, bbox_.pos_y_, bbox_.pos_z_));

  vis_publisher_.publish(marker);
}

void VisualizerRos1::publishMeshCallback(const ros::TimerEvent&)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = mesh_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "yak_ros";
  marker.id = 0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  //  marker.frame_locked = true;  // Should be enabled if the frame is moving

  marker.mesh_resource = mesh_resource_;
  vis_publisher_.publish(marker);
}
