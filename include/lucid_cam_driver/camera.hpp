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

// Global constants
constexpr PfncFormat_ PIXEL_FORMAT = BGR8;      // constexpr used like that are like #define but with a type allowing for checks
constexpr const char* FILE_NAME = "test.png";
constexpr uint64_t devUpdateTimeout = 1000;


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

public:
    CameraDriver(Logger* lg, bool _isMasterCam, bool* _extShouldStop, uint64_t timeout);
    ~CameraDriver();
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
    
    Arena::IDevice* GetDevice();
    Logger* GetLogger();
    std::queue<Arena::IImage*>& GetImageQueue();
    Arena::ISystem* GetSystem();
    void SetSystem(Arena::ISystem* pSystem);
    std::vector<Arena::DeviceInfo>& GetDeviceInfos();
    void SetDeviceInfos(std::vector<Arena::DeviceInfo> deviceInfos);
};