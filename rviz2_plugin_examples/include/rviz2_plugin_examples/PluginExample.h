//
// Created by bzeren on 31.01.2023.
//

#ifndef DISPLAY_WS_PLUGINEXAMPLE_H
#define DISPLAY_WS_PLUGINEXAMPLE_H

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rviz_common/ros_topic_display.hpp"

#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <rviz_2d_overlay_msgs/msg/pie_chart.hpp>
#include <rviz_2d_overlay_msgs/msg/plotter2_d.hpp>
#include "pie_chart_display.h"

#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class PluginExample : public rclcpp::Node {

public:
    PluginExample();

private:
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr rtkStatus;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr mErrorPubAvarage;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr mYawErrorPub;




    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_real_pose_sub;

    void gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_sub;

    void gnss_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped ::SharedPtr msg);
    void ndt_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped ::SharedPtr msg);

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr gnss_pose_error_;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr ndt_pose_error_;

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr gnss_orientation_error_;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr ndt_orientation_error_;

};


#endif //DISPLAY_WS_PLUGINEXAMPLE_H
