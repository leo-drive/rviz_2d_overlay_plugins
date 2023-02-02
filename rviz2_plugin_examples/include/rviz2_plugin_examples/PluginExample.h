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

class PluginExample : public rclcpp::Node {

public:
    PluginExample(int argc, char **argv);

private:
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr mOverlayPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mPieChartPublisher;

    rviz_2d_overlay_plugins::PieChartDisplay mPieChartDisplay;

    rclcpp::TimerBase::SharedPtr mTimer;
    void mTimerFunc();

    double mDummyError;
};


#endif //DISPLAY_WS_PLUGINEXAMPLE_H
