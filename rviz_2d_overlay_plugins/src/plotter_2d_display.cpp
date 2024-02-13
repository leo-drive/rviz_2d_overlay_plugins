//
// Created by bzeren on 24.02.2023.
//

#include "plotter_2d_display.hpp"

#include <OgreHardwarePixelBuffer.h>
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/display_context.hpp"
#include <QPainter>

namespace rviz_2d_overlay_plugins {
    Plotter2DDisplay::Plotter2DDisplay() : min_value_(0.0), max_value_(0.0) {

        show_value_property_ = new rviz_common::properties::BoolProperty("Show Value", true,
                                                                         "Show the current value as text.",
                                                                         this, SLOT(updateShowValue()));
        buffer_length_property_ = new rviz_common::properties::IntProperty("Buffer Length", 10000000000,
                                                                           "Length of the buffer used to store the values.",
                                                                           this, SLOT(updateBufferSize()));
        width_property_ = new rviz_common::properties::IntProperty("Width", 64,
                                                                   "Width of the plotter in pixels.",
                                                                   this, SLOT(updateWidth()));
        height_property_ = new rviz_common::properties::IntProperty("Height", 64,
                                                                    "Height of the plotter in pixels.",
                                                                    this, SLOT(updateHeight()));
        hor_dist_property_ = new rviz_common::properties::IntProperty("hor_dist", 0, "horizontal distance to anchor",
                                                                      this, SLOT(updateHorizontalDistance()));
        ver_dist_property_ = new rviz_common::properties::IntProperty("ver_dist", 0, "vertical distance to anchor",
                                                                      this, SLOT(updateVerticalDistance()));
        hor_alignment_property_ =
                new rviz_common::properties::EnumProperty("hor_alignment", "left",
                                                          "horizontal alignment of the overlay",
                                                          this, SLOT(updateHorizontalAlignment()));
        hor_alignment_property_->addOption("left", rviz_2d_overlay_msgs::msg::OverlayText::LEFT);
        hor_alignment_property_->addOption("center", rviz_2d_overlay_msgs::msg::OverlayText::CENTER);
        hor_alignment_property_->addOption("right", rviz_2d_overlay_msgs::msg::OverlayText::RIGHT);
        ver_alignment_property_ =
                new rviz_common::properties::EnumProperty("ver_alignment", "top", "vertical alignment of the overlay",
                                                          this, SLOT(updateVerticalAlignment()));
        ver_alignment_property_->addOption("top", rviz_2d_overlay_msgs::msg::OverlayText::TOP);
        ver_alignment_property_->addOption("center", rviz_2d_overlay_msgs::msg::OverlayText::CENTER);
        ver_alignment_property_->addOption("bottom", rviz_2d_overlay_msgs::msg::OverlayText::BOTTOM);
        auto_scale_property_ = new rviz_common::properties::BoolProperty("Auto Scale", false,
                                                                         "Automatically scale the plotter to the current value.",
                                                                         this, SLOT(updateAutoScale()));
        max_value_property_ = new rviz_common::properties::FloatProperty("Max Value", 100.0,
                                                                         "Maximum value of the plotter.",
                                                                         this, SLOT(updateMaxValue()));
        min_value_property_ = new rviz_common::properties::FloatProperty("Min Value", 0.0,
                                                                         "Minimum value of the plotter.",
                                                                         this, SLOT(updateMinValue()));
        fg_color_property_ = new rviz_common::properties::ColorProperty("Foreground Color", QColor(25, 255, 40),
                                                                        "Color of the plotter.",
                                                                        this, SLOT(updateFGColor()));
        fg_alpha_property_ = new rviz_common::properties::FloatProperty("Foreground Alpha", 1.0,
                                                                        "Alpha value of the plotter.",
                                                                        this, SLOT(updateFGAlpha()));
        fg_alpha_property_->setMin(0.0);
        fg_alpha_property_->setMax(1.0);
        bg_color_property_ = new rviz_common::properties::ColorProperty("Background Color", QColor(0, 0, 0),
                                                                        "Color of the background.",
                                                                        this, SLOT(updateBGColor()));
        bg_alpha_property_ = new rviz_common::properties::FloatProperty("Background Alpha", 0.0,
                                                                        "Alpha value of the background.",
                                                                        this, SLOT(updateBGAlpha()));
        bg_alpha_property_->setMin(0.0);
        bg_alpha_property_->setMax(1.0);
        line_width_property_ = new rviz_common::properties::IntProperty("Line Width", 1,
                                                                        "Width of the line used to draw the plotter.",
                                                                        this, SLOT(updateLineWidth()));
        line_width_property_->setMin(1);
        line_width_property_->setMax(1000);
        show_border_property_ = new rviz_common::properties::BoolProperty("Show Border", true,
                                                                          "Show a border around the plotter.",
                                                                          this, SLOT(updateShowBorder()));
        text_size_property_ = new rviz_common::properties::IntProperty("Text Size", 12,
                                                                       "Size of the text used to display the value.",
                                                                       this, SLOT(updateTextSize()));
        text_size_property_->setMin(1);
        text_size_property_->setMax(1000);
        show_caption_property_ = new rviz_common::properties::BoolProperty("Show Caption", true,
                                                                           "Show a caption above the plotter.",
                                                                           this, SLOT(updateShowCaption()));
        update_interval_property_ = new rviz_common::properties::FloatProperty("Update Interval", 0.04,
                                                                               "Interval in seconds between two updates.",
                                                                               this, SLOT(updateUpdateInterval()));
        update_interval_property_->setMin(0.0);
        update_interval_property_->setMax(100.0);
        auto_color_change_property_ = new rviz_common::properties::BoolProperty("Auto Color Change", false,
                                                                                "Automatically change the color of the plotter.",
                                                                                this, SLOT(updateAutoColorChange()));
        max_color_property_ = new rviz_common::properties::ColorProperty("Max Color", QColor(255, 0, 0),
                                                                         "Color of the plotter when the value is at its maximum.",
                                                                         this, SLOT(updateMaxColor()));
    caption_property_ = new rviz_common::properties::StringProperty("Caption", "",
                                                                    "Caption of the plotter.",
                                                                    this, SLOT(updateCaption()));
    }

    Plotter2DDisplay::~Plotter2DDisplay() {
        onDisable();
    }

    void Plotter2DDisplay::initializeBuffer() {
        buffer_.resize(buffer_length_);
        if (min_value_ == 0.0 && max_value_ == 0.0) {
            min_value_ = -1.0;
            max_value_ = 1.0;
        }
        for (size_t i = 0; i < buffer_length_; i++) {
            buffer_[i] = 0.0;
        }
    }

    void Plotter2DDisplay::onInitialize() {
        static int count = 0;
        RTDClass::onInitialize();
        rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
        rviz_common::UniformStringStream ss;
        ss << "Plotter2DDisplayObject" << count++;
        overlay_.reset(new OverlayObject(ss.str()));
        updateBufferSize();
        onEnable();
        updateTopic();
        updateShowValue();
        updateWidth();
        updateHeight();
        updateHorizontalDistance();
        updateVerticalDistance();
        updateHorizontalAlignment();
        updateVerticalAlignment();
        updateFGColor();
        updateBGColor();
        updateFGAlpha();
        updateBGAlpha();
        updateLineWidth();
        updateUpdateInterval();
        updateShowBorder();
        updateAutoColorChange();
        updateMaxColor();
        updateShowCaption();
        updateTextSize();
        updateAutoScale();
        updateMinValue();
        updateMaxValue();
        updateCaption();
        overlay_->updateTextureSize(width_property_->getInt(),
                                    height_property_->getInt() + caption_offset_);
        draw_required_ = true;
    }

    void Plotter2DDisplay::drawPlot() {
        QColor fg_color(fg_color_);
        QColor bg_color(bg_color_);

        fg_color.setAlpha(fg_alpha_);
        bg_color.setAlpha(bg_alpha_);

        if (auto_color_change_) {
            double r
                    = std::min(std::max((buffer_[buffer_.size() - 1] - min_value_) / (max_value_ - min_value_),
                                        0.0), 1.0);
            if (r > 0.3) {
                double r2 = (r - 0.3) / 0.7;
                fg_color.setRed((max_color_.red() - fg_color_.red()) * r2
                                + fg_color_.red());
                fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2
                                  + fg_color_.green());
                fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2
                                 + fg_color_.blue());
            }
        }

        {
            ScopedPixelBuffer buffer = overlay_->getBuffer();
            QImage Hud = buffer.getQImage(*overlay_);
            // initilize by the background color
            for (int i = 0; i < overlay_->getTextureWidth(); i++) {
                for (int j = 0; j < overlay_->getTextureHeight(); j++) {
                    Hud.setPixel(i, j, bg_color.rgba());
                }
            }
            // paste in HUD speedometer. I resize the image and offset it by 8 pixels from
            // the bottom left edge of the render window
            QPainter painter(&Hud);
            painter.setRenderHint(QPainter::Antialiasing, true);
            painter.setPen(QPen(fg_color, line_width_, Qt::SolidLine));

            uint16_t w = overlay_->getTextureWidth();
            uint16_t h = overlay_->getTextureHeight() - caption_offset_;

            double margined_max_value = max_value_ + (max_value_ - min_value_) / 2;
            double margined_min_value = min_value_ - (max_value_ - min_value_) / 2;

            for (size_t i = 1; i < buffer_length_; i++) {
                double v_prev = (margined_max_value - buffer_[i - 1]) / (margined_max_value - margined_min_value);
                double v = (margined_max_value - buffer_[i]) / (margined_max_value - margined_min_value);
                double u_prev = (i - 1) / (float) buffer_length_;
                double u = i / (float) buffer_length_;

                // chop within 0 ~ 1
                v_prev = std::max(std::min(v_prev, 1.0), 0.0);
                u_prev = std::max(std::min(u_prev, 1.0), 0.0);
                v = std::max(std::min(v, 1.0), 0.0);
                u = std::max(std::min(u, 1.0), 0.0);

                uint16_t x_prev = (int) (u_prev * w);
                uint16_t x = (int) (u * w);
                uint16_t y_prev = (int) (v_prev * h);
                uint16_t y = (int) (v * h);
                painter.drawLine(x_prev, y_prev, x, y);
            }
            // draw border
            if (show_border_) {
                painter.drawLine(0, 0, 0, h);
                painter.drawLine(0, h, w, h);
                painter.drawLine(w, h, w, 0);
                painter.drawLine(w, 0, 0, 0);
            }
            // draw caption
            if (show_caption_) {
                QFont font = painter.font();
                font.setPointSize(text_size_);
                font.setBold(true);
                painter.setFont(font);
                painter.drawText(0, h, w, caption_offset_,
                                 Qt::AlignCenter | Qt::AlignVCenter,
                                 caption_);
            }
            if (show_value_) {
                QFont font = painter.font();
                font.setPointSize(text_size_);
                font.setBold(true);
                painter.setFont(font);
                std::ostringstream ss;
                std::string unit_char = unit_.toUtf8().constData();
                ss << std::fixed << std::setprecision(4) << buffer_[buffer_.size() - 1] << " " << unit_char;
                painter.drawText(0, -20, w, h,
                                 Qt::AlignCenter | Qt::AlignVCenter,
                                 ss.str().c_str());
            }

            // done
            painter.end();
        }
    }

    void Plotter2DDisplay::processMessage(const rviz_2d_overlay_msgs::msg::Plotter2D::ConstSharedPtr msg) {
        boost::mutex::scoped_lock lock(mutex_);
        if (!isEnabled()) {
            return;
        }
        // add the message to the buffer
        double min_value = buffer_[0];
        double max_value = buffer_[0];
        for (size_t i = 0; i < buffer_length_ - 1; i++) {
            buffer_[i] = buffer_[i + 1];
            if (min_value > buffer_[i]) {
                min_value = buffer_[i];
            }
            if (max_value < buffer_[i]) {
                max_value = buffer_[i];
            }
        }
        buffer_[buffer_length_ - 1] = msg->data;
        if (min_value > msg->data) {
            min_value = msg->data;
        }
        if (max_value < msg->data) {
            max_value = msg->data;
        }
        if (auto_scale_) {
            min_value_ = min_value;
            max_value_ = max_value;
            if (min_value_ == max_value_) {
                min_value_ = min_value_ - 0.5;
                max_value_ = max_value_ + 0.5;
            }
        }
        if (!overlay_->isVisible()) {
            return;
        }

        horizontal_dist_ = msg->horizontal_distance;
        vertical_dist_ = msg->vertical_distance;
        horizontal_alignment_ = HorizontalAlignment{msg->horizontal_alignment};
        vertical_alignment_ = VerticalAlignment{msg->vertical_alignment};
        left_ = msg->horizontal_distance;
        top_ = msg->vertical_distance;
        caption_ = QString(msg->caption.c_str());
        unit_ = QString(msg->unit.c_str());
        fg_color_ = QColor(msg->fg_color.r * 255.0, msg->fg_color.g * 255.0, msg->fg_color.b * 255.0,
                           msg->fg_color.a * 255.0);
        texture_width_ = msg->width;
        texture_height_ = msg->height;

        max_value_ = msg->max_value;
        min_value_ = msg->min_value;

        draw_required_ = true;
    }

    void Plotter2DDisplay::update(float wall_dt, float ros_dt) {
        if (draw_required_) { // TODO: burası true olmuyor
            if (wall_dt + last_time_ > update_interval_) {
                overlay_->updateTextureSize(texture_width_,
                                            texture_height_ + caption_offset_);
                overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
                overlay_->setPosition(horizontal_dist_, vertical_dist_, horizontal_alignment_, vertical_alignment_);
                last_time_ = 0;
                drawPlot();
                draw_required_ = false;
            } else {
                last_time_ = last_time_ + wall_dt;
            }
        }
    }

    void Plotter2DDisplay::onEnable()
    {
        if (overlay_) {
            overlay_->show();
        }
        subscribe();
    }

    void Plotter2DDisplay::onDisable()
    {
        if (overlay_) {
            overlay_->hide();
        }
        unsubscribe();
    }

    void Plotter2DDisplay::updateWidth()
    {
        boost::mutex::scoped_lock lock(mutex_);
        texture_width_ = width_property_->getInt();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateHeight()
    {
        boost::mutex::scoped_lock lock(mutex_);
        texture_height_ = height_property_->getInt();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateVerticalDistance() {
        boost::mutex::scoped_lock lock(mutex_);
        vertical_dist_ = ver_dist_property_->getInt();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateHorizontalDistance() {
        boost::mutex::scoped_lock lock(mutex_);
        horizontal_dist_ = hor_dist_property_->getInt();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateVerticalAlignment() {
        boost::mutex::scoped_lock lock(mutex_);
        vertical_alignment_ = VerticalAlignment{static_cast<uint8_t>(ver_alignment_property_->getOptionInt())};
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateHorizontalAlignment() {
        boost::mutex::scoped_lock lock(mutex_);
        horizontal_alignment_ = HorizontalAlignment{static_cast<uint8_t>(hor_alignment_property_->getOptionInt())};
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateBGColor()
    {
        bg_color_ = bg_color_property_->getColor();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateFGColor()
    {
        fg_color_ = fg_color_property_->getColor();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateFGAlpha()
    {
        fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateBGAlpha()
    {
        bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateShowValue()
    {
        show_value_ = show_value_property_->getBool();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateShowBorder()
    {
        show_border_ = show_border_property_->getBool();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateLineWidth()
    {
        line_width_ = line_width_property_->getInt();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateBufferSize()
    {
        buffer_length_ = buffer_length_property_->getInt();
        initializeBuffer();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateAutoColorChange()
    {
        auto_color_change_ = auto_color_change_property_->getBool();
        if (auto_color_change_) {
            max_color_property_->show();
        }
        else {
            max_color_property_->hide();
        }
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateMaxColor()
    {
        max_color_ = max_color_property_->getColor();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateUpdateInterval()
    {
        update_interval_ = update_interval_property_->getFloat();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateTextSize()
    {
        text_size_ = text_size_property_->getInt();
        QFont font;
        font.setPointSize(text_size_);
        caption_offset_ = QFontMetrics(font).height();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateShowCaption()
    {
        show_caption_  = show_caption_property_->getBool();
        if (show_caption_) {
            text_size_property_->show();
        }
        else {
            text_size_property_->hide();
        }
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateMinValue()
    {
        if (!auto_scale_) {
            min_value_ = min_value_property_->getFloat();
        }
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateMaxValue()
    {
        if (!auto_scale_) {
            max_value_ = max_value_property_->getFloat();
        }
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateAutoScale()
    {
        auto_scale_ = auto_scale_property_->getBool();
        if (auto_scale_) {
            min_value_property_->hide();
            max_value_property_->hide();
        }
        else {
            min_value_property_->show();
            max_value_property_->show();
        }
        updateMinValue();
        updateMaxValue();
        draw_required_ = true;
    }

    void Plotter2DDisplay::updateCaption()
    {
        caption_ = caption_property_->getString();
        draw_required_ = true;
    }

//    bool Plotter2DDisplay::isInRegion(int x, int y)
//    {
//        return (top_ < y && top_ + texture_height_ > y &&
//                left_ < x && left_ + texture_width_ > x);
//    }
//
//    void Plotter2DDisplay::movePosition(int x, int y)
//    {
//        top_ = y;
//        left_ = x;
//    }
//
//    void Plotter2DDisplay::setPosition(int x, int y)
//    {
//        top_property_->setValue(y);
//        left_property_->setValue(x);
//    }

} // namespace rviz_2d_overlay_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_2d_overlay_plugins::Plotter2DDisplay, rviz_common::Display)