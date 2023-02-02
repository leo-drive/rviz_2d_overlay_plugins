//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"

#include "rviz2_plugin_examples/PluginExample.h"

PluginExample::PluginExample(int argc, char **argv) : Node("plugin_example"), mDummyError{0.0} {

    mOverlayPublisher = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("overlay_text", 10);
    mPieChartPublisher = create_publisher<std_msgs::msg::Float32>("pie_chart", 10);

    mTimer = create_wall_timer(std::chrono::milliseconds(500), std::bind(&PluginExample::mTimerFunc, this));;
}

void PluginExample::mTimerFunc() {

    rviz_2d_overlay_msgs::msg::OverlayText msg = rviz_2d_overlay_msgs::msg::OverlayText();

    msg.text = "Loc. Error: " + std::to_string(mDummyError);

    if (mDummyError <= 5.0) {
        mDummyError += 0.2;
    }

    double minError = 0.0;
    double maxError = 5.0;
    double temp = (((mDummyError - minError) / (maxError - minError)) * (1.0 - 0.0)) + 0.0;

    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float>(temp);
    color.g = 1.0f - static_cast<float>(temp);
    color.b = 0.0f;
    color.a = 1.0f;
    msg.fg_color = color;

    msg.height = static_cast<int32_t>(32); //
    msg.width = static_cast<int32_t>(172); // Kapladığı maksimum alanın boyutları

    msg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    msg.horizontal_distance = static_cast<int32_t>(32);

    msg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    msg.vertical_distance = static_cast<int32_t>(32);

    msg.text_size = static_cast<int32_t>(12);

    mOverlayPublisher->publish(msg);

    std_msgs::msg::Float32 pieChartMsg;
    pieChartMsg.data = static_cast<float>(temp);
    mPieChartPublisher->publish(pieChartMsg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>(argc, argv));
    rclcpp::shutdown();
    return 0;
}