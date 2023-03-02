//
// Created by bzeren on 24.02.2023.
//

#ifndef RVIZ_2D_OVERLAY_PLUGINS_PLOTTER_2D_DISPLAY_H
#define RVIZ_2D_OVERLAY_PLUGINS_PLOTTER_2D_DISPLAY_H

#include "rviz_2d_overlay_msgs/msg/plotter2_d.hpp"
#ifndef Q_MOC_RUN

#include <boost/thread/mutex.hpp>
#include <iomanip>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/render_system.hpp>
#include <std_msgs/msg/color_rgba.h>

#include "overlay_utils.hpp"

#endif

namespace rviz_2d_overlay_plugins {
    class Plotter2DDisplay : public rviz_common::RosTopicDisplay<rviz_2d_overlay_msgs::msg::Plotter2D> {
    Q_OBJECT
    public:
        Plotter2DDisplay();

        virtual ~Plotter2DDisplay();

    protected:
        rviz_2d_overlay_plugins::OverlayObject::SharedPtr overlay_;
        QColor fg_color_;
        QColor max_color_;
        QColor bg_color_;

        double fg_alpha_;
        double bg_alpha_;
        bool auto_scale_;
        bool show_border_;
        bool auto_color_change_;
        bool show_value_;
        bool show_caption_;
        bool draw_required_;
        float last_time_;
        float update_interval_;

        int buffer_length_;
        std::vector<double> buffer_;
        uint16_t texture_width_;
        uint16_t texture_height_;
        int horizontal_dist_;
        int vertical_dist_;
        HorizontalAlignment horizontal_alignment_;
        VerticalAlignment vertical_alignment_;
        int line_width_;
        int text_size_;
        int caption_offset_;
        double min_value_;
        double max_value_;
        QString caption_;
        QString unit_;
        int width_;
        int height_;
        int left_;
        int top_;

        bool overtake_position_properties_;
        bool require_update_texture_;

        virtual void update(float wall_dt, float ros_dt) override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void initializeBuffer();
        virtual void onInitialize() override;
        virtual void drawPlot();

        rviz_common::properties::BoolProperty *overtake_position_properties_property_;
        rviz_common::properties::BoolProperty *show_value_property_;
        rviz_common::properties::ColorProperty *fg_color_property_;
        rviz_common::properties::ColorProperty *bg_color_property_;
        rviz_common::properties::FloatProperty *fg_alpha_property_;
        rviz_common::properties::FloatProperty *bg_alpha_property_;
        rviz_common::properties::FloatProperty *update_interval_property_;
        rviz_common::properties::BoolProperty *show_border_property_;
        rviz_common::properties::IntProperty *buffer_length_property_;
        rviz_common::properties::IntProperty *width_property_;
        rviz_common::properties::IntProperty *height_property_;
        rviz_common::properties::IntProperty *hor_dist_property_;
        rviz_common::properties::IntProperty *ver_dist_property_;
        rviz_common::properties::EnumProperty *hor_alignment_property_;
        rviz_common::properties::EnumProperty *ver_alignment_property_;
        rviz_common::properties::IntProperty *line_width_property_;
        rviz_common::properties::BoolProperty *auto_color_change_property_;
        rviz_common::properties::ColorProperty *max_color_property_;
        rviz_common::properties::BoolProperty *show_caption_property_;
        rviz_common::properties::IntProperty *text_size_property_;
        rviz_common::properties::BoolProperty *auto_scale_property_;
        rviz_common::properties::FloatProperty *max_value_property_;
        rviz_common::properties::FloatProperty *min_value_property_;

        rviz_common::properties::IntProperty* left_property_;
        rviz_common::properties::IntProperty* top_property_;

        // ROS VARIABLES
        boost::mutex mutex_;

    protected Q_SLOTS:
        void updateShowValue();
        void updateBufferSize();
        void updateBGColor();
        void updateFGColor();
        void updateFGAlpha();
        void updateBGAlpha();
        void updateWidth();
        void updateHeight();
        void updateTop();
        void updateLeft();
        void updateHorizontalDistance();
        void updateVerticalDistance();
        void updateHorizontalAlignment();
        void updateVerticalAlignment();
        void updateLineWidth();
        void updateShowBorder();
        void updateAutoColorChange();
        void updateMaxColor();
        void updateUpdateInterval();
        void updateShowCaption();
        void updateTextSize();
        void updateAutoScale();
        void updateMinValue();
        void updateMaxValue();
    private:
        void processMessage(rviz_2d_overlay_msgs::msg::Plotter2D::ConstSharedPtr msg) override;
    };
} // namespace rviz_2d_overlay_plugins

#endif //RVIZ_2D_OVERLAY_PLUGINS_PLOTTER_2D_DISPLAY_H
