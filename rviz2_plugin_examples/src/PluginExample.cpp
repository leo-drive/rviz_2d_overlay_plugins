//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"
#include "math.h"
#include "string.h"

PluginExample::PluginExample() : Node("plugin_example") {

    ndt_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose_estimator/pose_with_covariance", 100,
            std::bind(&PluginExample::ndt_pose_callback, this, std::placeholders::_1));

    gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", 100,
            std::bind(&PluginExample::gnss_pose_callback, this, std::placeholders::_1));

    ekf_pose_source_sub_ = create_subscription<std_msgs::msg::String>(
            "/localization/autoware_pose_covariance_modifier/selected_pose_type", 100,
            std::bind(&PluginExample::ekf_pose_source_callback, this, std::placeholders::_1));

    gnss_position_rmse_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("gnss_position_error", 10);
    ndt_position_rmse_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("ndt_position_error", 10);

    gnss_yaw_rmse_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("gnss_yaw_error_in_degrees", 10);
    ndt_yaw_rmse_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("ndt_yaw_error_in_degrees", 10);

    ekf_pose_source_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("ekf_pose_source", 10);
}
void PluginExample::ekf_pose_source_callback(const std_msgs::msg::String::SharedPtr msg){
    std_msgs::msg::ColorRGBA color_fg;
    color_fg.r = 0.7f;
    color_fg.g = 0.8f;
    color_fg.b = 0.8f;
    color_fg.a = 1.0f;
    std_msgs::msg::ColorRGBA color_bg;
    color_bg.r = 1.0f;
    color_bg.g = 1.0f;
    color_bg.b = 1.0f;
    color_bg.a = 0.01f;

    rviz_2d_overlay_msgs::msg::OverlayText ekf_pose_source_plug_;
    ekf_pose_source_plug_.action = 0;
    ekf_pose_source_plug_.text = "EKF pose sources: \n" + msg->data;
    ekf_pose_source_plug_.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::RIGHT;
    ekf_pose_source_plug_.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    ekf_pose_source_plug_.fg_color = color_fg;
    ekf_pose_source_plug_.bg_color = color_bg;
    ekf_pose_source_plug_.width = 250;
    ekf_pose_source_plug_.height = 70;
    ekf_pose_source_plug_.text_size = 20;
    ekf_pose_source_plug_.line_width = 150;
    ekf_pose_source_plug_.horizontal_distance = static_cast<int32_t>(32);
    ekf_pose_source_plug_.vertical_distance = static_cast<int32_t>(32);
    ekf_pose_source_pub_->publish(ekf_pose_source_plug_);
}
void PluginExample::ndt_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    std_msgs::msg::ColorRGBA color;

    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 0.2f;
    color.a = 1.0f;

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg;
    double rmse_ = std::sqrt((msg->pose.covariance[0] + msg->pose.covariance[7]) / 2);
    plotterMsg.data = static_cast<float>(rmse_);
    plotterMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg.caption = "NDT POS RMSE (EKF)";
    plotterMsg.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg.vertical_distance = static_cast<int32_t>(352);
    plotterMsg.width = 200;
    plotterMsg.height = 128;
    plotterMsg.min_value = 0.0;
    plotterMsg.max_value = 1.0;
    plotterMsg.fg_color = color;
    plotterMsg.unit = "m";
    ndt_position_rmse_pub_->publish(plotterMsg);


    rviz_2d_overlay_msgs::msg::Plotter2D orientationPlotterMsg;
    double rmse_in_degrees = std::sqrt(msg->pose.covariance[35]) * 180 / M_PI;
    orientationPlotterMsg.data = static_cast<float>(rmse_in_degrees);
    orientationPlotterMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::RIGHT;
    orientationPlotterMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    orientationPlotterMsg.caption = "NDT Yaw RMSE (EKF)";
    orientationPlotterMsg.horizontal_distance = static_cast<int32_t>(32);
    orientationPlotterMsg.vertical_distance = static_cast<int32_t>(352);
    orientationPlotterMsg.width = 200;
    orientationPlotterMsg.height = 128;
    orientationPlotterMsg.min_value = 0.0;
    orientationPlotterMsg.max_value = 5.0;
    orientationPlotterMsg.fg_color = color;
    orientationPlotterMsg.unit = "degrees";
    ndt_yaw_rmse_pub_->publish(orientationPlotterMsg);

}

void PluginExample::gnss_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std_msgs::msg::ColorRGBA color;

    color.r = 1.0f;
    color.g = 0.25f;
    color.b = 0.0f;
    color.a = 1.0f;

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg;
    double rmse_ = std::sqrt((msg->pose.covariance[0] + msg->pose.covariance[7]) / 2);
    plotterMsg.data = static_cast<float>(rmse_);
    plotterMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg.caption = "GNSS POS RMSE (EKF)";
    plotterMsg.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg.vertical_distance = static_cast<int32_t>(192);
    plotterMsg.width = 200;
    plotterMsg.height = 128;
    plotterMsg.min_value = 0.0;
    plotterMsg.max_value = 1.0;
    plotterMsg.fg_color = color;
    plotterMsg.unit = "m";
    gnss_position_rmse_pub_->publish(plotterMsg);


    rviz_2d_overlay_msgs::msg::Plotter2D orientationPlotterMsg;
    double rmse_in_degrees = std::sqrt(msg->pose.covariance[35]) * 180 / M_PI;

    orientationPlotterMsg.data = static_cast<float>(rmse_in_degrees);
    orientationPlotterMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::RIGHT;
    orientationPlotterMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    orientationPlotterMsg.caption = "GNSS Yaw RMSE (EKF)";
    orientationPlotterMsg.horizontal_distance = static_cast<int32_t>(32);
    orientationPlotterMsg.vertical_distance = static_cast<int32_t>(192);
    orientationPlotterMsg.width = 200;
    orientationPlotterMsg.height = 128;
    orientationPlotterMsg.min_value = 0.0;
    orientationPlotterMsg.max_value = 5.0;
    orientationPlotterMsg.fg_color = color;
    orientationPlotterMsg.unit = "degrees";
    gnss_yaw_rmse_pub_->publish(orientationPlotterMsg);

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}
