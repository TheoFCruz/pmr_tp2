#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <eigen3/Eigen/Dense>

#include <map>
#include <string>
#include <vector>

class Visualizer
{
  using Point = geometry_msgs::msg::Point;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;
  using Marker = visualization_msgs::msg::Marker;

public:
  explicit Visualizer(rclcpp::Node *node)
  : node(node)
  {
    std::string topic = std::string(node->get_name()) + "/path";
    path_publisher = node->create_publisher<Path>(
      topic,
      rclcpp::QoS(1).transient_local().reliable()
    );
  }

  void publishPoint(
    const std::string &topic,
    const Eigen::Vector2d &point,
    const std::string &frame_id = "map",
    int id = 0,
    double r = 1.0,
    double g = 0.0,
    double b = 0.0)
  {
    auto publisher = getMarkerPublisher(topic);

    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.ns = topic;
    marker.id = id;
    marker.type = Marker::SPHERE;
    marker.action = Marker::ADD;

    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    publisher->publish(marker);
  }

  void publishPointsArray(
    const std::string &topic,
    const std::vector<Eigen::Vector2d> &points,
    const std::string &frame_id = "map",
    int id = 0,
    double r = 1.0,
    double g = 0.0,
    double b = 0.0)
  {
    auto publisher = getMarkerPublisher(topic);

    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.ns = topic;
    marker.id = id;
    marker.type = Marker::POINTS;
    marker.action = Marker::ADD;

    marker.scale.x = 0.12;
    marker.scale.y = 0.12;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    for (const auto& point_vector : points)
    {
      Point point;
      point.x = point_vector.x();
      point.y = point_vector.y();
      point.z = 0.0;
      marker.points.push_back(point);
    }

    publisher->publish(marker);
  }

  void publishPath(
    const std::vector<Eigen::Vector2d> &points,
    const std::string &frame_id = "map")
  {
    if (!path_publisher) return;

    Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = node->now();

    for (const auto& point : points)
    {
      PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_publisher->publish(path);
  }

private:
  rclcpp::Publisher<Marker>::SharedPtr getMarkerPublisher(
    const std::string &topic)
  {
    if (marker_publishers.count(topic) == 0)
    {
      marker_publishers[topic] =
        node->create_publisher<Marker>(topic, 10);
    }

    return marker_publishers[topic];
  }

  rclcpp::Node *node;

  // map topic to publisher
  std::map<std::string, rclcpp::Publisher<Marker>::SharedPtr> marker_publishers;
  rclcpp::Publisher<Path>::SharedPtr path_publisher;
};
