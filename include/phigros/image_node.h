//
// Created by baiye on 25-3-9.
//

#ifndef IMAGE_NODE_H
#define IMAGE_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

//ROS2节点，用于处理图像数据并检测蓝色音符及判定线，发布点击位置
class image_node : public rclcpp::Node {
public:
  image_node();

private:
  void find_blue(cv::Mat &mask, cv::Mat &image, std::vector<cv::Point> &object_points);

  void find_white(std::vector<cv::Vec4i> &lines, cv::Mat &image, cv::Vec4i &object_line);

  void click_blue(int y, std::vector<cv::Point> &object_points);

  void image_callback(sensor_msgs::msg::Image msg);


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr click_pub_;
};


#endif //IMAGE_NODE_H
