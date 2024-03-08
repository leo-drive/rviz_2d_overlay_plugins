//
// Created by bzeren on 31.01.2023.
//

#ifndef DISPLAY_WS_PLUGINEXAMPLE_H
#define DISPLAY_WS_PLUGINEXAMPLE_H

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  ekf_pose_source_sub_;

    void gnss_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void ndt_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void ekf_pose_source_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr gnss_position_rmse_pub_;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr ndt_position_rmse_pub_;

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr gnss_yaw_rmse_pub_;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr ndt_yaw_rmse_pub_;

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr ekf_pose_source_pub_;

};


#endif //DISPLAY_WS_PLUGINEXAMPLE_H
