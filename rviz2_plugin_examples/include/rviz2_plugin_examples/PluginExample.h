//
// Created by bzeren on 31.01.2023.
//

#ifndef DISPLAY_WS_PLUGINEXAMPLE_H
#define DISPLAY_WS_PLUGINEXAMPLE_H

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rviz_common/ros_topic_display.hpp"

#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <rviz_2d_overlay_msgs/msg/pie_chart.hpp>
#include <rviz_2d_overlay_msgs/msg/plotter2_d.hpp>
#include "pie_chart_display.h"

#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <applanix_msgs/msg/navigation_performance_gsof50.hpp>

class PluginExample : public rclcpp::Node {

public:
    PluginExample();

private:
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr mErrorPub1;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr mErrorPub2;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr mErrorPub3;

    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::Plotter2D>::SharedPtr mNdtTime;

    rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr ndt_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub;

    void ndt_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg);
    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};


#endif //DISPLAY_WS_PLUGINEXAMPLE_H
