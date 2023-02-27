//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"

PluginExample::PluginExample() : Node("plugin_example") {

    velocity_error_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("velocity_error", 10);
    orientation_error_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("orientation_error", 10);
    mPieChartPublisher = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("pie_chart", 10);
    plotterPublisher = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("plotter", 10);
    ndt_sub = create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
            "/localization/pose_estimator/exe_time_ms", 100,
            std::bind(&PluginExample::ndt_callback, this, std::placeholders::_1));
    gnss_sub = create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>(
            "/lvx_client/gsof/ins_solution_rms_50", 100,
            std::bind(&PluginExample::gnss_callback, this, std::placeholders::_1));
}

void PluginExample::ndt_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg) {

    double time_err_color_min = 20.0;
    double time_err_color_max = 70.0;
    double time_err_temp =
            (((msg->data - time_err_color_min) / (time_err_color_max - time_err_color_min)) * (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA color;
    if (msg->data < 20.0) {
        color.r = 0.0f;
        color.g = 1.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    } else if (20.0 < msg->data && msg->data < 100.0) {
        color.r = static_cast<float>(time_err_temp);
        color.g = 1.0f - static_cast<float>(time_err_temp);
        color.b = 0.0f;
        color.a = 1.0f;
    } else {
        color.r = 1.0f;
        color.g = 0.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    }

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg;
    plotterMsg.data = static_cast<float>(msg->data);
    plotterMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg.caption = "NDT Process Time";
    plotterMsg.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg.vertical_distance = static_cast<int32_t>(32);
    plotterMsg.width = 172;
    plotterMsg.height = 128;
    plotterMsg.min_value = 0.0;
    plotterMsg.max_value = 100.0;
    plotterMsg.fg_color = color;
    plotterPublisher->publish(plotterMsg);
}

void PluginExample::gnss_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg) {


    // [VELOCITY ERROR]
    double avg_vel_error =
            (fabs(msg->vel_rms_error.north) + fabs(msg->vel_rms_error.east) + fabs(msg->vel_rms_error.down)) / 3;

    double vel_err_color_min = 0.0045;
    double vel_err_color_max = 0.008;
    double vel_err_temp =
            (((avg_vel_error - vel_err_color_min) / (vel_err_color_max - vel_err_color_min)) * (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA vel_color;
    if (avg_vel_error < 0.0045) {
        vel_color.r = 0.0f;
        vel_color.g = 1.0f;
        vel_color.b = 0.0f;
        vel_color.a = 1.0f;
    } else if (0.004 < avg_vel_error && avg_vel_error < 0.008) {
        vel_color.r = static_cast<float>(vel_err_temp);
        vel_color.g = 1.0f - static_cast<float>(vel_err_temp);
        vel_color.b = 0.0f;
        vel_color.a = 1.0f;
    } else {
        vel_color.r = 1.0f;
        vel_color.g = 0.0f;
        vel_color.b = 0.0f;
        vel_color.a = 1.0f;
    }

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg;
    plotterMsg.data = static_cast<float>(avg_vel_error);
    plotterMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg.caption = "Linear Vel. Av. Err.";
    plotterMsg.horizontal_distance = static_cast<int32_t>(236);
    plotterMsg.vertical_distance = static_cast<int32_t>(32);
    plotterMsg.width = 172;
    plotterMsg.height = 128;
    plotterMsg.min_value = 0.0;
    plotterMsg.max_value = 0.012;
    plotterMsg.fg_color = vel_color;
    velocity_error_pub_->publish(plotterMsg);
    // [VELOCITY ERROR]

    // [ORIENTATION ERROR]
    double avg_orient_error = (fabs(msg->attitude_rms_error_heading) + fabs(msg->attitude_rms_error_pitch) +
                               fabs(msg->attitude_rms_error_roll)) / 3;

    double orient_err_color_min = 0.035;
    double orient_err_color_max = 0.08;
    double orient_err_temp =
            (((avg_orient_error - orient_err_color_min) / (orient_err_color_max - orient_err_color_min)) *
             (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA orient_color;
    if (avg_orient_error < 0.035) {
        orient_color.r = 0.0f;
        orient_color.g = 1.0f;
        orient_color.b = 0.0f;
        orient_color.a = 1.0f;
    } else if (0.035 < avg_orient_error && avg_orient_error < 0.08) {
        orient_color.r = static_cast<float>(orient_err_temp);
        orient_color.g = 1.0f - static_cast<float>(orient_err_temp);
        orient_color.b = 0.0f;
        orient_color.a = 1.0f;
    } else {
        orient_color.r = 1.0f;
        orient_color.g = 0.0f;
        orient_color.b = 0.0f;
        orient_color.a = 1.0f;
    }

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg2;
    plotterMsg2.data = static_cast<float>(avg_orient_error);
    plotterMsg2.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg2.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg2.caption = "Orientation Vel. Av. Err.";
    plotterMsg2.horizontal_distance = static_cast<int32_t>(440);
    plotterMsg2.vertical_distance = static_cast<int32_t>(32);
    plotterMsg2.width = 172;
    plotterMsg2.height = 128;
    plotterMsg2.min_value = 0.0;
    plotterMsg2.max_value = 0.12;
    plotterMsg2.fg_color = orient_color;
    orientation_error_pub_->publish(plotterMsg2);
    // [ORIENTATION ERROR]
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}
