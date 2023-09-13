//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"

PluginExample::PluginExample() : Node("plugin_example") {

    mErrorPub1 = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("error_1", 10);
    mErrorPub2 = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("error_2", 10);
    mErrorPub3 = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("error_3", 10);
    mErrorPubAvarage = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("avarage_position_error", 10);

    rtkStatus = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("rtk_status", 10);

    mNdtTime = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("ndt_time", 10);

    ndt_sub = create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
            "/localization/pose_estimator/exe_time_ms", 100,
            std::bind(&PluginExample::ndt_callback, this, std::placeholders::_1));
    gnss_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/clap/ros/gps_nav_sat_fix", 100,
            std::bind(&PluginExample::gnss_callback, this, std::placeholders::_1));
    rtk_sub = create_subscription<clap_b7_driver::msg::ClapIns>(
            "/sensing/gnss/clap/clap_ins", 100,
            std::bind(&PluginExample::rtk_callback, this, std::placeholders::_1));
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
    plotterMsg.caption = "NDT P. Time";
    plotterMsg.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg.vertical_distance = static_cast<int32_t>(32);
    plotterMsg.width = 172;
    plotterMsg.height = 128;
    plotterMsg.min_value = 0.0;
    plotterMsg.max_value = 100.0;
    plotterMsg.fg_color = color;
    plotterMsg.unit = "ms";
    mNdtTime->publish(plotterMsg);
}

void PluginExample::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

    double error_1 = std::sqrt(msg->position_covariance[0]);
    double min_error_1 = 1.0;
    double max_error_1 = 5.0;
    double error_1_temp =
            (((error_1 - min_error_1) / (max_error_1 - min_error_1)) * (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA color;
    if (error_1 < min_error_1) {
        color.r = 0.0f;
        color.g = 1.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    } else if (min_error_1 < error_1 && error_1 < max_error_1) {
        color.r = static_cast<float>(error_1_temp);
        color.g = 1.0f - static_cast<float>(error_1_temp);
        color.b = 0.0f;
        color.a = 1.0f;
    } else {
        color.r = 1.0f;
        color.g = 0.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    }
    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_1;
    plotterMsg_1.data = static_cast<float>(error_1);
    plotterMsg_1.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg_1.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_1.caption = "GNSS Error 1";
    plotterMsg_1.horizontal_distance = static_cast<int32_t>(236);
    plotterMsg_1.vertical_distance = static_cast<int32_t>(32);
    plotterMsg_1.width = 172;
    plotterMsg_1.height = 128;
    plotterMsg_1.min_value = 0.0;
    plotterMsg_1.max_value = 100.0;
    plotterMsg_1.fg_color = color;
    plotterMsg_1.unit = "m";
    mErrorPub1->publish(plotterMsg_1);

    double error_2 = std::sqrt(msg->position_covariance[4]);
    double min_error_2 = 1.0;
    double max_error_2 = 5.0;
    double error_2_temp =
            (((error_2 - min_error_2) / (max_error_2 - min_error_2)) * (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA color_2;
    if (error_2 < min_error_2) {
        color_2.r = 0.0f;
        color_2.g = 1.0f;
        color_2.b = 0.0f;
        color_2.a = 1.0f;
    } else if (min_error_2 < error_2 && error_2 < max_error_2) {
        color_2.r = static_cast<float>(error_2_temp);
        color_2.g = 1.0f - static_cast<float>(error_2_temp);
        color_2.b = 0.0f;
        color_2.a = 1.0f;
    } else {
        color_2.r = 1.0f;
        color_2.g = 0.0f;
        color_2.b = 0.0f;
        color_2.a = 1.0f;
    }
    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_2;
    plotterMsg_2.data = static_cast<float>(error_2);
    plotterMsg_2.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg_2.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_2.caption = "GNSS Error 2";
    plotterMsg_2.horizontal_distance = static_cast<int32_t>(440);
    plotterMsg_2.vertical_distance = static_cast<int32_t>(32);
    plotterMsg_2.width = 172;
    plotterMsg_2.height = 128;
    plotterMsg_2.min_value = 0.0;
    plotterMsg_2.max_value = 100.0;
    plotterMsg_2.fg_color = color_2;
    plotterMsg_2.unit = "m";
    mErrorPub2->publish(plotterMsg_2);

    double error_3 = std::sqrt(msg->position_covariance[8]);
    double min_error_3 = 1.0;
    double max_error_3 = 5.0;
    double error_3_temp =
            (((error_3 - min_error_3) / (max_error_3 - min_error_3)) * (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA color_3;
    if (error_3 < min_error_3) {
        color_3.r = 0.0f;
        color_3.g = 1.0f;
        color_3.b = 0.0f;
        color_3.a = 1.0f;
    } else if (min_error_3 < error_3 && error_3 < max_error_3) {
        color_3.r = static_cast<float>(error_3_temp);
        color_3.g = 1.0f - static_cast<float>(error_3_temp);
        color_3.b = 0.0f;
        color_3.a = 1.0f;
    } else {
        color_3.r = 1.0f;
        color_3.g = 0.0f;
        color_3.b = 0.0f;
        color_3.a = 1.0f;
    }

    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_3;
    plotterMsg_3.data = static_cast<float>(error_3);
    plotterMsg_3.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg_3.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_3.caption = "GNSS Error 3";
    plotterMsg_3.horizontal_distance = static_cast<int32_t>(644);
    plotterMsg_3.vertical_distance = static_cast<int32_t>(32);
    plotterMsg_3.width = 172;
    plotterMsg_3.height = 128;
    plotterMsg_3.min_value = 0.0;
    plotterMsg_3.max_value = 100.0;
    plotterMsg_3.fg_color = color_3;
    plotterMsg_3.unit = "m";
    mErrorPub3->publish(plotterMsg_3);

        double avarage_position_error = (error_1 + error_2 + error_3 )/ 3;
    double min_error_avarage = 1.0;
    double max_error_avarage = 5.0;
    double avarage_position_error_temp =
            (((avarage_position_error - min_error_avarage) / (max_error_avarage - min_error_avarage)) * (1.0 - 0.0)) + 0.0;
    std_msgs::msg::ColorRGBA color_avarage;
    if (avarage_position_error < min_error_avarage) {
        color_avarage.r = 0.0f;
        color_avarage.g = 1.0f;
        color_avarage.b = 0.0f;
        color_avarage.a = 1.0f;
    } else if (min_error_avarage < avarage_position_error && avarage_position_error < max_error_avarage) {
        color_avarage.r = static_cast<float>(avarage_position_error_temp);
        color_avarage.g = 1.0f - static_cast<float>(avarage_position_error_temp);
        color_avarage.b = 0.0f;
        color_avarage.a = 1.0f;
    } else {
        color_avarage.r = 1.0f;
        color_avarage.g = 0.0f;
        color_avarage.b = 0.0f;
        color_avarage.a = 1.0f;
    }
    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_avarage;
    plotterMsg_avarage.data = static_cast<float>(avarage_position_error);
    plotterMsg_avarage.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg_avarage.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_avarage.caption = "GNSS Error";
    plotterMsg_avarage.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg_avarage.vertical_distance = static_cast<int32_t>(200);
    plotterMsg_avarage.width = 172;
    plotterMsg_avarage.height = 128;
    plotterMsg_avarage.min_value = 0.0;
    plotterMsg_avarage.max_value = 100.0;
    plotterMsg_avarage.fg_color = color_3;
    plotterMsg_avarage.unit = "m";
    mErrorPubAvarage->publish(plotterMsg_avarage);

}

void PluginExample::rtk_callback(const clap_b7_driver::msg::ClapIns::SharedPtr msg) {
    std_msgs::msg::ColorRGBA color;
    std_msgs::msg::ColorRGBA color_bg;
    color_bg.r = 1.0f;
    color_bg.g = 1.0f;
    color_bg.b = 1.0f;
    color_bg.a = 1.0f;

    rviz_2d_overlay_msgs::msg::OverlayText textMsg;
    textMsg.action = 0;
    if(msg->pos_type ==0 ){
        textMsg.text = "GPS POSITION TYPE: \n NO_SOLUTION";
    }
    else if(msg->pos_type == 52){
        textMsg.text = "GPS POSITION TYPE: \n UNKNOWN_TYPE";
    }
    else if(msg->pos_type == 53){
        textMsg.text = "GPS POSITION TYPE: \n SINGLE";
    }
    else if(msg->pos_type == 54){
        textMsg.text = "GPS POSITION TYPE: \n DIFF";
    }
    else if(msg->pos_type == 55){
        textMsg.text = "GPS POSITION TYPE: \n RTK FLOAT";
    }
    else if(msg->pos_type == 56){
        textMsg.text = "GPS POSITION TYPE: \n RTK INT (FIXED)";
    }
//    textMsg.text=std::to_string(msg->status.type);
    textMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    textMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    textMsg.fg_color = color;
    textMsg.bg_color = color_bg;
    textMsg.width = 100;
    textMsg.height = 100;
    textMsg.text_size = 1000;
    rtkStatus->publish(textMsg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}
