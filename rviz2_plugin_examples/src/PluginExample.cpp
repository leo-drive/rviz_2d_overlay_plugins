//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"
#include "rviz2_plugin_examples/PluginExample.h"
#include <cstdlib>

PluginExample::PluginExample() : Node("plugin_example"){

    velocity_error_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("velocity_error", 10);
    orientation_error_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("orientation_error", 10);
    mPieChartPublisher = create_publisher<std_msgs::msg::Float32>("pie_chart", 10);
    ndt_sub=create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
        "/localization/pose_estimator/exe_time_ms", 100, std::bind(&PluginExample::ndt_callback, this, std::placeholders::_1));
    gnss_sub=create_subscription<applanix_msgs::msg::NavigationPerformanceGsof50>("/lvx_client/gsof/ins_solution_rms_50",100,std::bind(&PluginExample::gnss_callback,this,std::placeholders::_1));
}

void PluginExample::ndt_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg){
    std_msgs::msg::Float32 pieChartMsg;
    pieChartMsg.data = static_cast<float>(msg->data);
    mPieChartPublisher->publish(pieChartMsg);
}

void PluginExample::gnss_callback(const applanix_msgs::msg::NavigationPerformanceGsof50::SharedPtr msg){
    rviz_2d_overlay_msgs::msg::OverlayText velocity_err = rviz_2d_overlay_msgs::msg::OverlayText();   
    rviz_2d_overlay_msgs::msg::OverlayText orientation_err = rviz_2d_overlay_msgs::msg::OverlayText(); 

    double avg_vel_error=(fabs(msg->vel_rms_error.north)+fabs(msg->vel_rms_error.east)+fabs(msg->vel_rms_error.down))/3;
    double avg_orient_error=(fabs(msg->attitude_rms_error_heading)+fabs(msg->attitude_rms_error_pitch)+fabs(msg->attitude_rms_error_roll))/3;

    velocity_err.text = "Velocity average Error: " + std::to_string(avg_vel_error);
    orientation_err.text = "Orientation average Error: " + std::to_string(avg_orient_error);

    std_msgs::msg::ColorRGBA vel_color;
    if(avg_vel_error<0.004){
        vel_color.r = 0.0f;
        vel_color.g = 1.0f;
        vel_color.b = 0.0f;
        vel_color.a = 1.0f;
    }
    else{
        vel_color.r = 1.0f;
        vel_color.g = 0.0f;
        vel_color.b = 0.0f;
        vel_color.a = 1.0f;        
    }

    velocity_err.fg_color = vel_color;

    velocity_err.height = static_cast<int32_t>(32); //
    velocity_err.width = static_cast<int32_t>(500); // Kapladığı maksimum alanın boyutları

    velocity_err.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    velocity_err.horizontal_distance = static_cast<int32_t>(32);

    velocity_err.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    velocity_err.vertical_distance = static_cast<int32_t>(32);

    velocity_err.text_size = static_cast<int32_t>(12);

    std_msgs::msg::ColorRGBA orient_color;
    if(avg_orient_error<0.4){
        orient_color.r = 0.0f;
        orient_color.g = 1.0f;
        orient_color.b = 0.0f;
        orient_color.a = 1.0f;
    }
    else{
        orient_color.r = 1.0f;
        orient_color.g = 0.0f;
        orient_color.b = 0.0f;
        orient_color.a = 1.0f;        
    }
    orientation_err.fg_color = orient_color;

    orientation_err.height = static_cast<int32_t>(32); //
    orientation_err.width = static_cast<int32_t>(500); // Kapladığı maksimum alanın boyutları

    orientation_err.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::RIGHT;
    orientation_err.horizontal_distance = static_cast<int32_t>(32);

    orientation_err.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    orientation_err.vertical_distance = static_cast<int32_t>(32);

    orientation_err.text_size = static_cast<int32_t>(12);

    velocity_error_pub_->publish(velocity_err);
    orientation_error_pub_->publish(orientation_err);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}