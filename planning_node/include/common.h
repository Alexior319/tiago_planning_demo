#ifndef PLANNING_COMMON_H
#define PLANNING_COMMON_H

#include <fmt/format.h>

#define debug_info(...) ROS_INFO("%s", fmt::format(__VA_ARGS__).c_str())
#define debug_error(...) ROS_ERROR("%s", fmt::format(__VA_ARGS__).c_str())
#define debug_warn(...) ROS_WARN("%s", fmt::format(__VA_ARGS__).c_str())

#define ros_info(...) ROS_INFO("%s:%d: %s", __FILE__, __LINE__, fmt::format(__VA_ARGS__).c_str())
#define ros_error(...) ROS_ERROR("%s:%d: %s", __FILE__, __LINE__, fmt::format(__VA_ARGS__).c_str())
#define ros_warn(...) ROS_WARN("%s:%d: %s", __FILE__, __LINE__, fmt::format(__VA_ARGS__).c_str())


#endif