/**
 * @file camera.cpp
 * @author tlr
 * @brief Implementation of the LUCID Triton camera Driver with multi camera setup in mind
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */


// User includes
#include <lucid_cam_driver/camera.hpp>

//using namespace std::chrono_literals;
#define BINTYPE "Sum"

/// @brief Static member initialization for CameraDriver
std::mutex CameraDriver::_queueWrite;

/**
 * @brief Construct a new Camera Driver:: Camera Driver pObject
 * 
 * @param lg Logger to be used within the instance
 * @param _isMasterCam Determines the setup behavior of the camera, only one master should be on the whole system
 * @param extShouldStop Boolean to inform the looping threads they need to stop
 * @param timeout How long to wait for an image before throwing an error
 */
CameraDriver::CameraDriver(Logger* lg, bool _isMasterCam, bool* extShouldStop, uint64_t timeout)
: _timeout(timeout), _trigParams{}, _pSystem(nullptr), _pDevice(nullptr),  _pHwTriggerScriptCmd(nullptr), _lg(lg),_isMasterCam(_isMasterCam), _extShouldStop(extShouldStop) {

}

/**
 * @brief Destroy the Camera Driver and cleans the device
 * 
 */
CameraDriver::~CameraDriver() {
    _pSystem->DestroyDevice(_pDevice);
    if(_isMasterCam) {
        Arena::CloseSystem(_pSystem);
    }

}

/**
 * @brief Creates the device
 * 
 * @param deviceIndex Which camera should the device be mad with, only one device possible per camera
 */
void CameraDriver::SetupCamera(uint8_t deviceIndex) {
        if(_isMasterCam) {
            _pSystem = Arena::OpenSystem();
            _pSystem->UpdateDevices(devUpdateTimeout);
            _deviceInfos = _pSystem->GetDevices();
        }
        if (_deviceInfos.size() == 0) throw std::runtime_error(std::string("No camera connected"));
        if(deviceIndex > _deviceInfos.size()) throw std::runtime_error(std::string("Requested index unavailable"));
        _pDevice = _pSystem->CreateDevice(_deviceInfos[deviceIndex]);

}

/**
 * @brief Configures the trigger
 * @todo Proper error handling
 * 
 * @return false No error
 * @return true Shouldn't ever happen
 */
bool CameraDriver::ConfigureTrigger() {
    // save previous trigger settings
    _trigParams.triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerSelector");
    _trigParams.triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerMode");
    _trigParams.triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerSource");

    // Set trigger action
    Arena::SetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerSelector", "FrameStart");
    // Enable trigger
    Arena::SetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerMode", "On");
    // Set trigger source
    Arena::SetNodeValue<GenICam::gcstring>( _pDevice->GetNodeMap(), "TriggerSource", "Line0");      // green wire
    Arena::SetNodeValue<GenICam::gcstring>( _pDevice->GetNodeMap(), "TriggerActivation", "RisingEdge");

    Arena::SetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerOverlap", "PreviousFrame");  // Important to reach decent fps
    return false;
}

/**
 * @brief Removes the trigger and restaures the parametres captured in ConfigureTrigger
 * @todo Proper error handling
 * 
 * @return false No error
 * @return true Shouldn't ever happen
 */
bool CameraDriver::RemoveTrigger() {
    Arena::SetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerSource", _trigParams.triggerSourceInitial);
	Arena::SetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerMode", _trigParams.triggerModeInitial);
	Arena::SetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "TriggerSelector", _trigParams.triggerSelectorInitial);
    return false;
}


/**
 * @brief Sends a software trigger to the camera afer a delay
 * 
 * @param delay Delay to wait before triggering in seconds
 */
void CameraDriver::SoftwareTrigger(std::chrono::seconds delay) {
    std::this_thread::sleep_for(delay);
    std::cout << "trigger set" << std::endl;
    Arena::ExecuteNode(_pDevice->GetNodeMap(),"TriggerSoftware");
}

/**
 * @brief Optimizes the speed at which data should be sent and allows resend, some settings are to be changed on the host device too for good performances
 * 
 */
void CameraDriver::SetNetworkSettings() {
    // Enable stream auto negotiate packet size
	Arena::SetNodeValue<bool>(_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

	// Enable stream packet resend
	Arena::SetNodeValue<bool>(_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
}

/**
 * @brief Starts the hardware trigger
 * @todo Actually having something to trigger
 * @todo Move to the Camera master
 * 
 */
void CameraDriver::HWTrigger() {
    _serial = new SerialComm("/dev/ttyACM0");
    _serial->SendCommand(SET_TRIG_FREQ, 20);
    _serial->SendCommand(START_TRIG);
}

/**
 * @brief The big main function: Aquires images and send them immediately to a processing thread to be ready for next frame as fast as possible
 * @attention Should be run in a thread
 * @todo More error handling e.g timeouts, queue getting too big, camera disconnects
 * 
 * @param pObj Pointer to the camera instance it's working for
 */
void CameraDriver::AquireTriggeredImageLoop(CameraDriver* pObj){
    pObj->_pDevice->StartStream();
    
    // Waits for the trigger to be armed (Camera ready)
    bool triggerArmed = false;
	do {
		triggerArmed = Arena::GetNodeValue<bool>(pObj->_pDevice->GetNodeMap(), "TriggerArmed");
	} while (triggerArmed == false);

    if(pObj->_extShouldStop == nullptr) throw std::runtime_error("Stopping boolean poiting to nullptr");

    while(!(*pObj->_extShouldStop)) {
        Bench bba("Buffer aquisition", pObj->_lg);   // Measures how long it takes to make receive an image and start a new thread

        Arena::IImage* buff = pObj->_pDevice->GetImage(pObj->_timeout);        // Blocks the execution until triggered or timeout
        Logger::Log(LogLevel::DEBUG, "image dimensions: (%lux%lu)", buff->GetWidth(), buff->GetHeight());

        if(buff->HasImageData()) {
            std::thread imgProcessing(CameraDriver::ProcessImage, pObj, buff);
            imgProcessing.detach();
        }
        else {
            ASYNC_LOG_SIMPLE(LogLevel::WARNING, "Frame dropped (buffer empty/no image data)");
        }
    }

    pObj->_pDevice->StopStream();
}

/**
 * @brief Copies the image and queues it
 * 
 * @param pObj Pointer to the camera instance it's working for
 * @param buff Image buffer
 */
void CameraDriver::ProcessImage(CameraDriver* pObj, Arena::IImage* pbuff) {
    Bench bip("Image processing", pObj->GetLogger());

    Arena::IImage* img = Arena::ImageFactory::Copy(pbuff);   // Copy so we can give the buffer back to the Arena engine as soon as possible

    std::unique_lock<std::mutex> lc(_queueWrite);   // Queues aren't guaranteed thread safe on write
    pObj->_images.push(img);
    lc.unlock();

    pObj->GetDevice()->RequeueBuffer(pbuff);
}

/**
 * @brief Saves the image buffer in a directory as a PNG
 * @deprecated
 * 
 * @param pImage Buffer of the image
 * @param dirName Directory path for the images to be saved
 */
void CameraDriver::SaveImagePng(Arena::IImage* pImage, const char* dirName) {
    static uint32_t count;
    char imgName[128];
    sprintf(imgName, "%simg%d.png", dirName, count++);
    auto pConverted = Arena::ImageFactory::Convert(pImage, BGR8,Arena::DirectionalInterpolation);
    Save::ImageParams params(pConverted->GetWidth(), pConverted->GetHeight(), pConverted->GetBitsPerPixel());
    Save::ImageWriter writer(params, imgName);
    writer.SetPng(".png", 0, false);
    writer << pConverted->GetData();
    _lg->Log(LogLevel::INFO, "Saved: %simg%d.png", dirName, count++);

    Arena::ImageFactory::Destroy(pConverted);
}

/**
 * @brief Signal handler
 * @deprecated
 * 
 * @param s Signal value
 */
void CameraDriver::CtrlCHandler(int s) {
    _lg->Log(INFO, "ctrl + C triggered: %d", s);
}

/**
 * @brief Returns the internal device
 * 
 * @return Arena::IDevice* 
 */
Arena::IDevice* CameraDriver::GetDevice() {
    return _pDevice;
}

/**
 * @brief Returns the internal saved logger
 * 
 * @return Logger* 
 */
Logger* CameraDriver::GetLogger() {
    return _lg;
}

/**
 * @brief Prints a few random infos
 * 
 */
void CameraDriver::LogInfos() {
    try
    {
        ASYNC_LOG(LogLevel::INFO, "Link speed: %d\n", Arena::GetNodeValue<int64_t>(_pDevice->GetNodeMap(), "DeviceLinkSpeed"));
        ASYNC_LOG(LogLevel::INFO, "Bit depth: %s\n", Arena::GetNodeValue<GenICam::gcstring>(_pDevice->GetNodeMap(), "ADCBitDepth").c_str());
    }
    catch(GenICam::GenericException& ge)
    {
        std::cerr << ge.what() << '\n';
    }
}

/**
 * @brief Returns a reference to the internal image queue
 * 
 * @return std::queue<Arena::IImage*>& 
 */
std::queue<Arena::IImage*>& CameraDriver::GetImageQueue() {
    return _images;
}

/**
 * @brief Returns the internal system
 * 
 * @return Arena::ISystem* 
 */
Arena::ISystem* CameraDriver::GetSystem() {
    return _pSystem;
}

/**
 * @brief Sets the internal system
 * 
 * @param pSystem Already initialized system
 */
void CameraDriver::SetSystem(Arena::ISystem* pSystem) {
    _pSystem = pSystem;
}

/**
 * @brief Returns the internal device info vector
 * 
 * @return std::vector<Arena::DeviceInfo>& 
 */
std::vector<Arena::DeviceInfo>& CameraDriver::GetDeviceInfos() {
    return _deviceInfos;
}

/**
 * @brief Sets the internal device info vector
 * 
 * @param deviceInfos Already filled vector
 */
void CameraDriver::SetDeviceInfos(std::vector<Arena::DeviceInfo> deviceInfos) {
    _deviceInfos = deviceInfos;
}

void CameraDriver::SetAllParams() {
#define X(field, type) SET_PARAMS(np, field, type);
    PARAM_FIELDS_ARENA
#undef X
#define X(field, type) SET_PARAMS_DOGSHITSTRINGS(np, field, type);
    PARAM_FIELDS_ARENA_DOGSHITSTRINGS
#undef X
}