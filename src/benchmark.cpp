/**
 * @file benchmark.cpp
 * @author tlr
 * @brief Implementation of the minimal benchmark class
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// User includes
#include <lucid_cam_driver/benchmark.hpp>


/**
 * @brief Sets the first time point on constructor
 * 
 * @param name Name used to display the message of the destructor
 */
Bench::Bench(const char* name): _name(name) {
    _start = std::chrono::high_resolution_clock::now();
}

/**
 * @brief Sets the first time point on constructor
 * 
 * @param name Name used to display the message of the destructor
 * @param lg Logger on which it'll display the destructor's message
 */
Bench::Bench(const char* name, Logger* lg): _name(name), _lg(lg) {
    _start = std::chrono::high_resolution_clock::now();
}

/**
 * @brief Display a message of how long the object was alive
 * 
 */
Bench::~Bench() {
    if(_lg != nullptr) {
        _lg->Log(DEBUG, "%s took %ld us\n\n", _name, GetUsTime());
    }
    else std::cout << _name << " took " << GetUsTime() << "us\n\n";
}

/**
 * @brief Returns the number of nanoseconds elapsed since instanciation
 * 
 * @return int64_t nanoseconds
 */
int64_t Bench::GetNsTime() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _start).count();
}

/**
 * @brief Returns the number of microseconds elapsed since instanciation
 * 
 * @return int64_t microseconds
 */
int64_t Bench::GetUsTime() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - _start).count();
}

/**
 * @brief Returns the number of milliseconds elapsed since instanciation
 * 
 * @return int64_t milliseconds
 */
int64_t Bench::GetMsTime() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - _start).count();
}

/**
 * @brief Returns the name of the object
 * 
 * @return const char* Name
 */
const char* Bench::GetName() {
    return _name;
}