#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <eigen3/Eigen/Dense>

#include <map>
#include <string>
#include <vector>

class Visualizer
{
  using Point = geometry_msgs::msg::Point;
  using Marker = visualization_msgs::msg::Marker;

public:
  explicit Visualizer(rclcpp::Node *node)
  : node(node)
  {
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
    double r = 0.7,
    double g = 0.7,
    double b = 0.7,
    double z = 0.12)
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
      point.z = z;
      marker.points.push_back(point);
    }

    publisher->publish(marker);
  }

  void publishLineStrip(
    const std::string &topic,
    const std::vector<Eigen::Vector2d> &points,
    const std::string &frame_id = "map",
    double r = 1.0,
    double g = 0.0,
    double b = 0.0,
    double width = 0.1
  )
  {
    auto publisher = getMarkerPublisher(topic);

    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.ns = topic;
    marker.id = 0;
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    for (const auto& point : points)
    {
      Point marker_point;
      marker_point.x = point.x();
      marker_point.y = point.y();
      marker_point.z = 0.12;
      marker.points.push_back(marker_point);
    }

    publisher->publish(marker);
  }

  void publishLineList(
    const std::string &topic,
    const std::vector<std::vector<Eigen::Vector2d>> &paths,
    const std::string &frame_id = "map",
    int id = 0,
    double r = 0.0,
    double g = 1.0,
    double b = 0.0,
    double a = 1.0,
    double width = 0.05,
    double z = 0.08)
  {
    auto publisher = getMarkerPublisher(topic);

    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.ns = topic;
    marker.id = id;
    marker.type = Marker::LINE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = width;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    for (const auto &path : paths)
    {
      for (std::size_t i = 1; i < path.size(); ++i)
      {
        Point start;
        start.x = path[i - 1].x();
        start.y = path[i - 1].y();
        start.z = z;
        marker.points.push_back(start);

        Point end;
        end.x = path[i].x();
        end.y = path[i].y();
        end.z = z;
        marker.points.push_back(end);
      }
    }

    publisher->publish(marker);
  }

  void publishCells(
    const std::string &topic,
    const std::vector<int> &cells,
    int map_width,
    double map_origin_x,
    double map_origin_y,
    double cell_width,
    const std::string &frame_id = "map",
    int id = 0,
    double r = 0.0,
    double g = 1.0,
    double b = 1.0,
    double a = 1.0,
    double z = 0.04)
  {
    auto publisher = getMarkerPublisher(topic);

    Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node->now();
    marker.ns = topic;
    marker.id = id;
    marker.type = Marker::POINTS;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = cell_width;
    marker.scale.y = cell_width;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    for (int index : cells)
    {
      Point point;
      point.x = map_origin_x + (index % map_width + 0.5) * cell_width;
      point.y = map_origin_y + (index / map_width + 0.5) * cell_width;
      point.z = z;
      marker.points.push_back(point);
    }

    publisher->publish(marker);
  }

private:
  rclcpp::Publisher<Marker>::SharedPtr getMarkerPublisher(
    const std::string &topic)
  {
    if (marker_publishers.count(topic) == 0)
    {
      marker_publishers[topic] =
        node->create_publisher<Marker>(
          topic,
          rclcpp::QoS(1).transient_local().reliable()
        );
    }

    return marker_publishers[topic];
  }

  rclcpp::Node *node;

  // map topic to publisher
  std::map<std::string, rclcpp::Publisher<Marker>::SharedPtr> marker_publishers;
};
