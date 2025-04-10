/**
 * @file logger.hpp
 * @author tlr
 * @brief A simple logger class
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// Stl includes
#include <iostream>
#include <fstream>
#include <atomic>
#include <mutex>
#include <cstdarg>

// Macros
// Very shady way of making all that async
#define ASYNC_LOG(x, y, ... ) std::thread([&] { Logger::Log(x, y, __VA_ARGS__);}).detach();
#define ASYNC_LOG_SIMPLE(x, y) std::thread([&] { Logger::Log(x, y);}).detach();

// Global constants
constexpr size_t MAX_BUFF = 256;

enum CCodes {
    FG_BLACK = 30,
    FG_RED = 31,
    FG_GREEN = 32,
    FG_YELLOW = 33,
    FG_BLUE = 34,
    FG_MAGENTA = 35,
    FG_CYAN = 36,
    FG_WHITE = 37,
    BG_BLACK = 40,
    BG_RED = 41,
    BG_GREEN = 42,
    BG_YELLOW = 43,
    BG_BLUE = 44,
    BG_MAGENTA = 45,
    BG_CYAN = 46,
    BG_WHITE = 47,
};

enum LogLevel {
    ERROR = 0,
    WARNING,
    INFO,
    DEBUG
};

class Color {
public:
    Color(CCodes code);
    friend std::ostream& operator<<(std::ostream& os, const Color& col);
private:
    CCodes _col;
};

class Logger {
private:
    static const char* _logFilePath;
    static LogLevel _loggingLevel;
    static LogLevel _defaultLoggingLevel;
    static bool _stdoutSupress;
    static const Color _levelColors[4];
    static std::ofstream logFile;

    static std::mutex _mtx;
public:
    Logger(const char* logFilePath, LogLevel loggingLevel, LogLevel defaultLoggingLevel);
    ~Logger();
    void Log(std::string s);
    static void Log(LogLevel l, std::string s);
    static void Log(LogLevel l, const char* format, ...);
    void LogError(std::string s);
    void LogWarning(std::string s);
    void LogInfo(std::string s);
    void LogDebug(std::string s);
    void SetLogFile(const char* logFilePath);
    void SupressStdout();
    void RestoreStdout();
};