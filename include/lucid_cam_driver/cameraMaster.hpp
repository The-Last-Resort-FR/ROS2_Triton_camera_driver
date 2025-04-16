/**
 * @file cameraMaster.hpp
 * @author tlr
 * @brief Manages one or more cameraDrivers and interfaces them with ROS2
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// Stl includes
#include <thread>
#include <vector>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include <image_transport/image_transport.hpp>

// User includes
#include <lucid_cam_driver/camera.hpp>
#include <lucid_cam_driver/logger.hpp>

// Global constants
constexpr uint16_t QUEUE_SIZE = 1;

/**
 * @brief Set the master for 1, 2 or more cameras
 * 
 */
enum MasterMode {
    SINGLE = 0,
    DUAL,
    MORE
};


class CameraMaster : public ::rclcpp::Node {
private:
    NodeParameters params;
    std::vector<CameraDriver*> _cams;
    Logger* _lg;
    bool _isValid;
    bool _shouldStop; 
    MasterMode _mode;
    uint8_t _camCount;
    rclcpp::NodeOptions _options;
    rclcpp::Node::SharedPtr _node;
    image_transport::ImageTransport* _it;
    std::vector<image_transport::Publisher> _publishers;
    public:
    CameraMaster(MasterMode mode, Logger* lg);
    ~CameraMaster();
    void DeclareAllParameters();
    void SetAllParameters();
    void StartCams();
    void Run();
};