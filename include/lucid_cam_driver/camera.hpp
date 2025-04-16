/**
 * @file camera.hpp
 * @author tlr
 * @brief a LUCID Triton camera Driver with multi camera setup in mind
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// Stl includes
#include <iostream>
#include <chrono>
#include <thread>
#include <exception>
#include <queue>
#include <mutex>

// Arena includes
#include <Arena/ArenaApi.h>
#include <Save/SaveApi.h>

// User includes
#include <lucid_cam_driver/logger.hpp>
#include <lucid_cam_driver/benchmark.hpp>
#include <lucid_cam_driver/serial.hpp>

// Macros
#define DECLARE_PARAM(obj, field) this->declare_parameter(#field, obj.field)
#define GET_PARAMS(obj, field) this->get_parameter(#field, obj.field)
#define SET_PARAMS(obj, field, type) Arena::SetNodeValue<type>( _pDevice->GetNodeMap(), #field, obj.field)
#define SET_PARAMS_DOGSHITSTRINGS(obj, field, type) Arena::SetNodeValue<type>( _pDevice->GetNodeMap(), #field, GenICam::gcstring(obj.field.c_str()))

// Some have a capital first letter to be the exact parameter name used in Arena
#define PARAM_FIELDS_DEC \
    X(mode, uint8_t)\
    X(StreamAutoNegotiatePacketSize, bool) \
    X(StreamPacketResendEnable, bool) \
    X(TriggerSelector, std::string) \
    X(TriggerMode, std::string) \
    X(TriggerSource, std::string) \
    X(TriggerActivation, std::string) \
    X(TriggerOverlap, std::string) \
    X(Line_mode, std::string) \
    X(Exposure_auto, std::string) \
    X(Exposure_time, double) \
    X(Gain, double) \
    X(Width, int64_t) \
    X(Height, int64_t) \
    X(Offset_x, int64_t) \
    X(Offset_y, int64_t) \
    X(PixelFormat, std::string)


// Same deal but only keeping the ones being set by Arena::SetNodeValue<>()
#define PARAM_FIELDS_ARENA \
    X(StreamAutoNegotiatePacketSize, bool) \
    X(StreamPacketResendEnable, bool) \
    X(Exposure_time, double) \
    X(Gain, double) \
    X(Width, int64_t) \
    X(Height, int64_t) \
    X(Offset_x, int64_t) \
    X(Offset_y, int64_t) \


#define PARAM_FIELDS_ARENA_DOGSHITSTRINGS \
    X(TriggerSelector, GenICam::gcstring) \
    X(TriggerMode, GenICam::gcstring) \
    X(TriggerSource, GenICam::gcstring) \
    X(TriggerActivation, GenICam::gcstring) \
    X(TriggerOverlap, GenICam::gcstring) \
    X(Line_mode, GenICam::gcstring) \
    X(Exposure_auto, GenICam::gcstring) \
    X(PixelFormat, GenICam::gcstring)


// Global constants
constexpr PfncFormat_ PIXEL_FORMAT = BGR8;      // constexpr used like that are like #define but with a type allowing for checks
constexpr const char* FILE_NAME = "test.png";
constexpr uint64_t devUpdateTimeout = 1000;


#define X(name, type) type name;
struct NodeParameters {
    PARAM_FIELDS_DEC
};
#undef X

class CameraDriver {
private:
    struct TriggerParams {
        GenICam::gcstring triggerSelectorInitial;
        GenICam::gcstring triggerModeInitial;
        GenICam::gcstring triggerSourceInitial;
    };

    uint64_t _timeout;
    Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo> _deviceInfos);
    TriggerParams _trigParams;
    Arena::ISystem* _pSystem;
    Arena::IDevice* _pDevice;
    const char* _pHwTriggerScriptCmd;
    Logger* _lg;
    bool _isMasterCam;
    bool* _extShouldStop;
    std::queue<Arena::IImage*> _images;
    static std::mutex _queueWrite;
    std::vector<Arena::DeviceInfo> _deviceInfos;
    SerialComm* _serial;

public:
    CameraDriver(Logger* lg, bool _isMasterCam, bool* _extShouldStop, uint64_t timeout);
    ~CameraDriver();
    NodeParameters np;
    void SetupCamera(uint8_t deviceIndex);
    bool ConfigureTrigger();
    bool RemoveTrigger();
    void LogInfos();
    void SoftwareTrigger(std::chrono::seconds delay);   //thread
    void SetNetworkSettings();
    void HWTrigger();   //thread
    static void AquireTriggeredImageLoop(CameraDriver* obj);    //main thread
    static void ProcessImage(CameraDriver* obj, Arena::IImage* buff);
    void SaveImagePng(Arena::IImage* pImage, const char* dirName);
    void CtrlCHandler(int s);
    void SetAllParams();
    
    Arena::IDevice* GetDevice();
    Logger* GetLogger();
    std::queue<Arena::IImage*>& GetImageQueue();
    Arena::ISystem* GetSystem();
    void SetSystem(Arena::ISystem* pSystem);
    std::vector<Arena::DeviceInfo>& GetDeviceInfos();
    void SetDeviceInfos(std::vector<Arena::DeviceInfo> deviceInfos);
};