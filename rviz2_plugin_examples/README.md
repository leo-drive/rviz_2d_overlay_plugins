## <div align="center">RVIZ 2D Plugin Examples</div>

---

## Text Overlay

![Alt Text](https://github.com/leo-drive/rviz_2d_overlay_plugins/tree/example_plugins/rviz2_plugin_examples/doc/text.gif)

### **_Usage:_**

Create a publisher which publishes a `rviz_2d_overlay_msgs::msg::OverlayText` message and publish it.



### **_Message Content:_**
| Parameter              |                                                    |
|:-----------------------|:---------------------------------------------------|
| `text`                 | `text to display`                                  |
| `height`               | `size of max. region`                              |
| `width`                | `size of max. region`                              |
| `horizontal_alignment` | `LEFT, RIGHT, CENTER, TOP, BOTTOM`                 |
| `vertical_alignment`   | `LEFT, RIGHT, CENTER, TOP, BOTTOM`                 |
| `horizontal_distance`  | `padding in horizontal`                            |
| `vertical_distance`    | `padding in vertical`                              |
| `color`                | `color message with type std_msgs::msg::ColorRGBA` |
| `text_size`            | `text size`                                        |


## Pie Chart Display

![Alt Text](https://github.com/leo-drive/rviz_2d_overlay_plugins/tree/example_plugins/rviz2_plugin_examples/doc/pieChart.gif)

Create a publisher which publishes a `std_msgs/Float32` message and publish it. On Rviz, add a Pie Chart Display and set the topic to the topic you published the message to.

> **_NOTE:_**  For displaying the plugins, you have to source rviz_2d_overlay_plugins and rviz_2d_overlay_msgs packages in your terminal which you are using for rviz2.