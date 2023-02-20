cmake_minimum_required(VERSION 3.5)
project(rviz2_plugin_examples)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(rviz_2d_overlay_plugins REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(applanix_msgs REQUIRED)

set (msg_file 
msg/Float32Stamped.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_file}
  DEPENDENCIES
    std_msgs
    builtin_interfaces
)

include_directories(
        include
        ${rviz_2d_overlay_plugins_INCLUDE_DIRS}
)

add_executable(plugin_example
        src/PluginExample.cpp
        )

target_link_libraries(plugin_example
        )

ament_target_dependencies(plugin_example
        rclcpp
        rviz_2d_overlay_msgs
        rviz_2d_overlay_plugins
        sensor_msgs
        tier4_debug_msgs
        applanix_msgs
        )


install(TARGETS plugin_example
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        DESTINATION share/${PROJECT_NAME}/
        )

ament_package()