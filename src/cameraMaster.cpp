/**
 * @file cameraMaster.cpp
 * @author tlr
 * @brief Implements the camera manager
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */


// User includes
#include <lucid_cam_driver/cameraMaster.hpp>

/**
 * @brief Construct a new Camera Master and creates all the cameras with their associated topics
 * 
 * @param mode Number of cameras
 * @param lg Logger to use internally
 */
CameraMaster::CameraMaster(MasterMode mode, Logger* lg): _lg(lg), _isValid(true), _shouldStop(false), _mode(mode), rclcpp::Node("camera_master") {
    _node = rclcpp::Node::make_shared("image_publisher", _options);
    _it = new image_transport::ImageTransport(_node);
    switch (_mode) {
        case SINGLE:
            _camCount = 1;
            _cams.push_back(new CameraDriver(_lg, true, &_shouldStop, 5000));
            _publishers.push_back(_it->advertise("camera/image", QUEUE_SIZE));
            break;

        case DUAL:
            _camCount = 2;
            _cams.push_back(new CameraDriver(_lg, true, &_shouldStop, 5000));
            _cams.push_back(new CameraDriver(_lg, false, &_shouldStop, 5000));
            _publishers.push_back(_it->advertise("camera/imageL", QUEUE_SIZE));
            _publishers.push_back(_it->advertise("camera/imageR", QUEUE_SIZE));
            break;
        
        default:
            _camCount = _mode + 1;
            for(uint8_t i = 0; i < _camCount; i++) {
                _cams.push_back(new CameraDriver(_lg, i ? false : true , &_shouldStop, 5000));
                char* publisherName;
                sprintf(publisherName, "camera/image%d", i);
                _publishers.push_back(_it->advertise(publisherName, QUEUE_SIZE));
            }
            ASYNC_LOG_SIMPLE(LogLevel::WARNING, "Untested mode : make sure to take in consideration the required bandwidth and processing power for such configuration");
            break;
    }
    StartCams();
    Run();

}

/**
 * @brief Sets should stop and wait for 500ms for threads to shutdown before deleting all the camera instances
 * 
 */
CameraMaster::~CameraMaster() {
    _shouldStop = true;
    std::this_thread::sleep_for(std::chrono::milliseconds (500));
    for (CameraDriver* i : _cams)
    {
        delete(i);
    }
}

/**
 * @brief Main loop that launches all the cameras' main thread, converts the images from the cameras' queues and publish them on their topic
 * 
 */
void CameraMaster::Run() {
    if(!_isValid) throw std::runtime_error("Invalide state");
    uint64_t id = 0;
    for(CameraDriver* cam : _cams) {
        std::thread c(CameraDriver::AquireTriggeredImageLoop, cam);
        c.detach();
    }
    while(!_shouldStop) {
        for(uint8_t i = 0; i < _camCount; i++) {
            if(_cams[i]->GetImageQueue().size() > 0) {
                char ids[40];
                std_msgs::msg::Header hdr;
                snprintf(ids, 40, "id%ld", id);
                hdr.stamp = this->now();
                hdr.set__frame_id(ids);
                Arena::IImage* img = _cams[i]->GetImageQueue().front();
                _cams[i]->GetImageQueue().pop();
                cv::Mat image_cv = cv::Mat(img->GetHeight(), img->GetWidth(), CV_8UC1, (uint8_t *)img->GetData());
                cv::Mat image_bgr(image_cv.rows, image_cv.cols, CV_8UC3);
                cvtColor(image_cv, image_bgr, cv::COLOR_BayerBG2BGR);
                cv::Mat msg_img = image_bgr.clone();
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", msg_img).toImageMsg();
                _publishers[i].publish(msg);
                Logger::Log(LogLevel::DEBUG, "images in queue: %d", _cams[i]->GetImageQueue().size());
                Arena::ImageFactory::Destroy(img);
            }
        }
    }
}

/**
 * @brief Run each camera setup
 * 
 */
void CameraMaster::StartCams() {
    uint8_t tmpCount = 0;
    try {
        switch (_mode) {
            case SINGLE:
                if(_camCount != 1) throw std::runtime_error("Invalid number of camera, corruption ?");
                _cams[0]->SetupCamera(0);
                _cams[0]->ConfigureTrigger();
                ASYNC_LOG(LogLevel::INFO, "%d cameras setup\n", _camCount);
                break;

            case DUAL:
                if(_camCount != 2) throw std::runtime_error("Invalid number of camera, corruption ?");
                for(CameraDriver* cam : _cams) {
                    if(tmpCount > 0) {
                        cam->SetSystem(_cams[0]->GetSystem());
                        cam->SetDeviceInfos(_cams[0]->GetDeviceInfos());
                    }
                    cam->SetupCamera(tmpCount++);
                    cam->ConfigureTrigger();
                    cam->SetNetworkSettings();
                }
                ASYNC_LOG(LogLevel::INFO, "%d cameras setup\n", _camCount);
                break;
            
            default:
                for(CameraDriver* cam : _cams) {
                    if(tmpCount > 0) {
                        cam->SetSystem(_cams[0]->GetSystem());
                        cam->SetDeviceInfos(_cams[0]->GetDeviceInfos());
                    }
                    cam->SetupCamera(tmpCount++);
                    cam->ConfigureTrigger();
                    cam->SetNetworkSettings();
                }
                ASYNC_LOG(LogLevel::INFO, "%d cameras setup\n", _camCount);
                break;
        }
    }
    catch (GenICam::GenericException& ge) {
        Logger::Log(LogLevel::ERROR, "GenICam exception thrown: %s\n", ge.what());
        _isValid = false;
    }
    catch (std::exception& ex) {
        Logger::Log(LogLevel::ERROR, "Standard exception thrown: %s\n", ex.what());
        _isValid = false;
    }
    catch (...) {
        Logger::Log(LogLevel::ERROR, "Unexpected exception thrown\n");
        _isValid = false;
    }
}

void CameraMaster::DeclareAllParameters() {
#define X(field, type) DECLARE_PARAM(params, field);
    PARAM_FIELDS_DEC
#undef X
}

void CameraMaster::SetAllParameters() {
#define X(field, type) GET_PARAMS(params, field);
    PARAM_FIELDS_DEC
#undef X
}