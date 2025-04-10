/**
 * @file benchmark.hpp
 * @author tlr
 * @brief A minimal benchmark class
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#pragma once

// Stl includes
#include <chrono>
#include <iostream>

// User includes
#include <lucid_cam_driver/logger.hpp>

class Bench {
private:
    std::chrono::high_resolution_clock::time_point _start;
    const char* _name;
    Logger* _lg;
public:
    Bench(const char* name);
    Bench(const char* name, Logger* lg);
    int64_t GetNsTime();
    int64_t GetUsTime();
    int64_t GetMsTime();
    const char* GetName();
    ~Bench();
};