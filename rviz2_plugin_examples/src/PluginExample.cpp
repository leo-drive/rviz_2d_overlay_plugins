//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"
#include "math.h"
#include "string.h"

PluginExample::PluginExample() : Node("plugin_example") {
    rtkStatus = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("RTK_Status", 10);

    mErrorPubAvarage = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("avarage_position_error", 10);
    mYawErrorPub = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("yaw_error_gnss",10);

    gnss_real_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance", 100,
            std::bind(&PluginExample::gnss_callback, this, std::placeholders::_1));

    ndt_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose_estimator/pose_with_covariance_w_switch", 100,
            std::bind(&PluginExample::ndt_pose_callback, this, std::placeholders::_1));

    gnss_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensing/gnss/pose_with_covariance_w_switch", 100,
            std::bind(&PluginExample::gnss_pose_callback, this, std::placeholders::_1));

    gnss_pose_error_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("gnss_position_error", 10);
    ndt_pose_error_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("ndt_position_error", 10);

    gnss_orientation_error_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("gnss_orientation_error_in_degrees", 10);
    ndt_orientation_error_ = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("ndt_orientation_error_in_degrees", 10);
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
    ndt_pose_error_->publish(plotterMsg);


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
    ndt_orientation_error_->publish(orientationPlotterMsg);

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
    gnss_pose_error_->publish(plotterMsg);


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
    gnss_orientation_error_->publish(orientationPlotterMsg);

}
void PluginExample::gnss_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    double avarage_position_error = (std::sqrt(msg->pose.covariance[0]) + std::sqrt(msg->pose.covariance[7]) ) / 2;
    std_msgs::msg::ColorRGBA color_real_gnss_;
    color_real_gnss_.r = 0.5f;
    color_real_gnss_.g = 0.5f;
    color_real_gnss_.b = 0.5f;
    color_real_gnss_.a = 1.0f;

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_avarage;
    plotterMsg_avarage.data = static_cast<float>(avarage_position_error);
    plotterMsg_avarage.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg_avarage.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_avarage.caption = "GNSS POS RMSE";
    plotterMsg_avarage.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg_avarage.vertical_distance = static_cast<int32_t>(32);
    plotterMsg_avarage.width = 200;
    plotterMsg_avarage.height = 128;
    plotterMsg_avarage.min_value = 0.0;
    plotterMsg_avarage.max_value = 1.0;
    plotterMsg_avarage.fg_color = color_real_gnss_;
    plotterMsg_avarage.unit = "m";
    mErrorPubAvarage->publish(plotterMsg_avarage);


    double yaw_error = std::sqrt(msg->pose.covariance[35]) * 180 / M_PI;

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_yaw;
    plotterMsg_yaw.data = static_cast<float>(yaw_error);
    plotterMsg_yaw.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::RIGHT;
    plotterMsg_yaw.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_yaw.caption = "GNSS Yaw RMSE";
    plotterMsg_yaw.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg_yaw.vertical_distance = static_cast<int32_t>(32);
    plotterMsg_yaw.width = 200;
    plotterMsg_yaw.height = 128;
    plotterMsg_yaw.min_value = 0.0;
    plotterMsg_yaw.max_value = 5.0;
    plotterMsg_yaw.fg_color = color_real_gnss_;
    plotterMsg_yaw.unit = "degrees";
    mYawErrorPub->publish(plotterMsg_yaw);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}
