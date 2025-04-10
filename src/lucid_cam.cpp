/**
 * @file lucid_cam.cpp
 * @author tlr
 * @brief Entry point
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// Stl includes
#include <memory>

// ROS includes
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

// User includes
#include <lucid_cam_driver/logger.hpp>
#include <lucid_cam_driver/benchmark.hpp>
#include <lucid_cam_driver/cameraMaster.hpp>



int main(int argc, char ** argv)
{
  Logger lg("misc/log.txt", DEBUG, INFO); // Must be set of the macro to work properly
  rclcpp::init(argc, argv);
  ASYNC_LOG_SIMPLE(LogLevel::INFO, "test");
  rclcpp::spin(std::make_shared<CameraMaster>(DUAL, &lg));  // Not sure how useful it is

  return 0;
}