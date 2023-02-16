//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"

#include "rviz2_plugin_examples/PluginExample.h"

PluginExample::PluginExample() : Node("plugin_example"){

    mOverlayPublisher = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("overlay_text", 10);
    mPieChartPublisher = create_publisher<std_msgs::msg::Float32>("pie_chart", 10);
    ndt_sub=create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
        "/localization/pose_estimator/exe_time_ms", 100, std::bind(&PluginExample::ndt_callback, this, std::placeholders::_1));
}

void PluginExample::ndt_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg){
    rviz_2d_overlay_msgs::msg::OverlayText overlay_msg = rviz_2d_overlay_msgs::msg::OverlayText();   

    overlay_msg.text = "Loc. Error: " + std::to_string(msg->data);

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0f;
    color.g = 1.0f;
    color.b = 0.0f;
    color.a = 1.0f;
    overlay_msg.fg_color = color;

    overlay_msg.height = static_cast<int32_t>(32); //
    overlay_msg.width = static_cast<int32_t>(172); // Kapladığı maksimum alanın boyutları

    overlay_msg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    overlay_msg.horizontal_distance = static_cast<int32_t>(32);

    overlay_msg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    overlay_msg.vertical_distance = static_cast<int32_t>(32);

    overlay_msg.text_size = static_cast<int32_t>(12);

    mOverlayPublisher->publish(overlay_msg);

    std_msgs::msg::Float32 pieChartMsg;
    pieChartMsg.data = static_cast<float>(msg->data);
    mPieChartPublisher->publish(pieChartMsg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}