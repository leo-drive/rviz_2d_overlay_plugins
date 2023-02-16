//
// Created by bzeren on 31.01.2023.
//

#ifndef DISPLAY_WS_PLUGINEXAMPLE_H
#define DISPLAY_WS_PLUGINEXAMPLE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include "rviz_common/ros_topic_display.hpp"
#include "pie_chart_display.h"
#include <tier4_debug_msgs/msg/float32_stamped.hpp>


class PluginExample : public rclcpp::Node {

public:
    PluginExample();

private:
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr mOverlayPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mPieChartPublisher;
    rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr ndt_sub;

    rviz_2d_overlay_plugins::PieChartDisplay mPieChartDisplay;
    
    void ndt_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg);
};


#endif //DISPLAY_WS_PLUGINEXAMPLE_H
