//
// Created by bzeren on 31.01.2023.
//

#include "rviz2_plugin_examples/PluginExample.h"

PluginExample::PluginExample() : Node("plugin_example") {

    mNdtTimePub = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("ndt_time_plot", 10);
    mGnssErrorPub = create_publisher<rviz_2d_overlay_msgs::msg::Plotter2D>("gnss_error_pub", 10);
    mRtkStatusPub = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("rtk_status_pub", 10);

    this->declare_parameter("ndt_time_topic", "");
    this->declare_parameter("nav_sat_fix_topic", "");
    this->declare_parameter("gpgga_topic", "");

    std::string ndt_time_topic;
    std::string nav_sat_fix_topic;
    std::string gpgga_topic;

    this->get_parameter("ndt_time_topic", ndt_time_topic);
    this->get_parameter("nav_sat_fix_topic", nav_sat_fix_topic);
    this->get_parameter("gpgga_topic", gpgga_topic);

    mNdtTimeSub = create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
            ndt_time_topic.c_str(), 100,
            std::bind(&PluginExample::ndt_callback, this, std::placeholders::_1));
    mGnssSub = create_subscription<sensor_msgs::msg::NavSatFix>(
            nav_sat_fix_topic.c_str(), 100,
            std::bind(&PluginExample::gnss_callback, this, std::placeholders::_1));
    mGpggaSub = create_subscription<nmea_msgs::msg::Gpgga>(
            gpgga_topic.c_str(), 100,
            std::bind(&PluginExample::gpgga_callback, this, std::placeholders::_1));
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
    mNdtTimePub->publish(plotterMsg);
}

void PluginExample::gnss_callback(const sensor_msgs::msg::NavSatFix &msg) {

    double average_position_error = (std::sqrt(msg.position_covariance[0]) + std::sqrt(msg.position_covariance[4]) +
                                     std::sqrt(msg.position_covariance[8])) / 3;
    double min_error_average = 1.0;
    double max_error_average = 5.0;
    double average_position_error_temp =
            (((average_position_error - min_error_average) / (max_error_average - min_error_average)) * (1.0 - 0.0)) +
            0.0;
    std_msgs::msg::ColorRGBA color;
    if (average_position_error < min_error_average) {
        color.r = 0.0f;
        color.g = 1.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    } else if (min_error_average < average_position_error && average_position_error < max_error_average) {
        color.r = static_cast<float>(average_position_error_temp);
        color.g = 1.0f - static_cast<float>(average_position_error_temp);
        color.b = 0.0f;
        color.a = 1.0f;
    } else {
        color.r = 1.0f;
        color.g = 0.0f;
        color.b = 0.0f;
        color.a = 1.0f;
    }
    rviz_2d_overlay_msgs::msg::Plotter2D plotterMsg_average;
    plotterMsg_average.data = static_cast<float>(average_position_error);
    plotterMsg_average.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    plotterMsg_average.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    plotterMsg_average.caption = "GNSS Error";
    plotterMsg_average.horizontal_distance = static_cast<int32_t>(32);
    plotterMsg_average.vertical_distance = static_cast<int32_t>(200);
    plotterMsg_average.width = 172;
    plotterMsg_average.height = 128;
    plotterMsg_average.min_value = 0.0;
    plotterMsg_average.max_value = 100.0;
    plotterMsg_average.fg_color = color;
    plotterMsg_average.unit = "m";
    mGnssErrorPub->publish(plotterMsg_average);
}

void PluginExample::gpgga_callback(const nmea_msgs::msg::Gpgga &msg){
    // RTK STATUS
    std_msgs::msg::ColorRGBA color_status;
    std_msgs::msg::ColorRGBA color_bg;
    color_bg.r = 1.0f;
    color_bg.g = 1.0f;
    color_bg.b = 1.0f;
    color_bg.a = 1.0f;
    rviz_2d_overlay_msgs::msg::OverlayText textMsg;
    textMsg.action = 0;
    if(msg.gps_qual ==0 ){
        textMsg.text = "GPS POSITION TYPE: \n NO_SOLUTION";
    }
    else if(msg.gps_qual == 1){
        textMsg.text = "GPS POSITION TYPE: \n UNKNOWN_TYPE";
    }
    else if(msg.gps_qual == 2){
        textMsg.text = "GPS POSITION TYPE: \n SINGLE";
    }
    else if(msg.gps_qual == 3){
        textMsg.text = "GPS POSITION TYPE: \n PSRDIFF";
    }
    else if(msg.gps_qual == 4){
        textMsg.text = "GPS POSITION TYPE: \n SBAS";
    }
    else if(msg.gps_qual == 5){
        textMsg.text = "GPS POSITION TYPE: \n OMNISTAR";
    }
    else if(msg.gps_qual == 6){
        textMsg.text = "GPS POSITION TYPE: \n RTK_FLOAT";
    }
    else if(msg.gps_qual == 7){
        textMsg.text = "GPS POSITION TYPE: \n RTK_INT";
    }
    else if(msg.gps_qual == 8){
        textMsg.text = "GPS POSITION TYPE: \n PPP_FLOAT";
    }
    else if(msg.gps_qual == 9){
        textMsg.text = "GPS POSITION TYPE: \n PPP_INT";
    }
    else if(msg.gps_qual == 10){
        textMsg.text = "GPS POSITION TYPE: \n FIXED";
    }
    textMsg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    textMsg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM;
    textMsg.fg_color = color_status;
    textMsg.bg_color = color_bg;
    textMsg.width = 100;
    textMsg.height = 100;
    textMsg.text_size = 1000;
    mRtkStatusPub->publish(textMsg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PluginExample>());
    rclcpp::shutdown();
    return 0;
}
