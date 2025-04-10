/**
 * @file logger.cpp
 * @author tlr
 * @brief Implementation of simple logger class
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// User includes
#include <lucid_cam_driver/logger.hpp>

/**
 * @brief Construct a new Color:: Color object
 * 
 * @param code Code from CCodes
 */
Color::Color(CCodes code): _col(code) {

}

/**
 * @brief Operator to print a special code to change the color in the terminal
 * 
 * @param os Stream
 * @param col The color to change the terminal text to
 * @return std::ostream& 
 */
std::ostream& operator<<(std::ostream& os, const Color& col) {
    return os << "\033[" << col._col << "m";
}

/// @brief Static member initialization for Logger 
const char* Logger::_logFilePath;

/// @brief Static member initialization for Logger 
LogLevel Logger::_loggingLevel;

/// @brief Static member initialization for Logger 
LogLevel Logger::_defaultLoggingLevel;

/// @brief Static member initialization for Logger 
bool Logger::_stdoutSupress = false;

/// @brief Static member initialization for Logger 
const Color Logger::_levelColors[4]{Color(CCodes::FG_RED), Color(CCodes::FG_YELLOW), Color(CCodes::FG_CYAN), Color(CCodes::FG_WHITE)};

/// @brief Static member initialization for Logger 
std::ofstream Logger::logFile;

/// @brief Static member initialization for Logger 
std::mutex Logger::_mtx;


/**
 * @brief Construct a new Logger:: Logger object
 * 
 * @param logFilePath Path to the file to output the logs, no file log if NULL
 * @param loggingLevel Level which severity should the Logger log things, discards the rest
 * @param defaultLoggingLevel Logging level when not specifying a log level
 */
Logger::Logger(const char* logFilePath, LogLevel loggingLevel, LogLevel defaultLoggingLevel) {
    _loggingLevel = loggingLevel;
    _logFilePath = logFilePath;
    _defaultLoggingLevel = defaultLoggingLevel;

    if(_logFilePath) {
        logFile.open(_logFilePath);
    }
    if(_defaultLoggingLevel > _loggingLevel) {
        std::cout << _levelColors[WARNING] << "[WARNING]: Default logging level lower than the globale logging level" << std::endl;
    }
}

/**
 * @brief Destroy the Logger:: Logger object
 * 
 */
Logger::~Logger() {
    if(logFile.is_open()) {
        logFile.close();
    }
}

/**
 * @brief Logs at default log level
 * 
 * @param s String to be logged
 */
void Logger::Log(std::string s) {
    Log(_defaultLoggingLevel, s);
}

/**
 * @brief Discards log if the level is bellow the instance's logging level
 * 
 * @param l Level of log
 * @param format format printf style
 * @param ... 
 */
void Logger::Log(LogLevel l, const char* format, ...){
    std::unique_lock<std::mutex> lock(_mtx);

    char buff[MAX_BUFF];
    va_list args;
    va_start(args, format);
    vsnprintf(buff, MAX_BUFF, format, args);
    va_end(args);

    if (l > _loggingLevel) return;
    if(!_stdoutSupress) {
        std::cout << _levelColors[l];
        switch (l) {
        case ERROR:
            std::cout << "[ERROR]:   ";
            break;
    
        case WARNING:
            std::cout << "[WARNING]: ";
            break;
    
        case INFO:
            std::cout << "[INFO]:    ";
            break;
    
        case DEBUG:
            std::cout << "[DEBUG]:   ";
            break;
        
        default:
            std::cout << "[ERROR]:   Log level not found\n";
            return;
        }
    
        std::cout << buff << std::endl;
    }

    if(logFile.is_open()) {
        logFile << _levelColors[l];
        switch (l) {
        case ERROR:
            logFile << "[ERROR]:   ";
            break;
    
        case WARNING:
            logFile << "[WARNING]: ";
            break;
    
        case INFO:
            logFile << "[INFO]:    ";
            break;
    
        case DEBUG:
            logFile << "[DEBUG]:   ";
            break;
        
        default:
            logFile << "[ERROR]:   Log level not found\n";
            return;
        }
    
        logFile << buff << std::endl;
    }
}

/**
 * @brief Discards log if the level is bellow the instance's logging level
 * 
 * @param l Log level
 * @param s String to log
 */
void Logger::Log(LogLevel l, std::string s){
    if (l > _loggingLevel) return;
    if(!_stdoutSupress) {
        std::cout << _levelColors[l];
        switch (l) {
        case ERROR:
            std::cout << "[ERROR]:   ";
            break;
    
        case WARNING:
            std::cout << "[WARNING]: ";
            break;
    
        case INFO:
            std::cout << "[INFO]:    ";
            break;
    
        case DEBUG:
            std::cout << "[DEBUG]:   ";
            break;
        
        default:
            std::cout << "[ERROR]:   Log level not found\n";
            return;
        }
    
        std::cout << s << std::endl;
    }

    if(logFile.is_open()) {
        logFile << _levelColors[l];
        switch (l) {
        case ERROR:
            logFile << "[ERROR]:   ";
            break;
    
        case WARNING:
            logFile << "[WARNING]: ";
            break;
    
        case INFO:
            logFile << "[INFO]:    ";
            break;
    
        case DEBUG:
            logFile << "[DEBUG]:   ";
            break;
        
        default:
            logFile << "[ERROR]:   Log level not found\n";
            return;
        }
    
        logFile << s << std::endl;
    }
}

/**
 * @brief Initialize or set a new log file
 * @todo Fix it
 * 
 * @param logFilePath New path of the log file
 */
void Logger::SetLogFile(const char* logFilePath) {
    _logFilePath = logFilePath;
    if(!logFile.is_open()) logFile.open(_logFilePath);
    else LogError("Invalide log file");
}

/**
 * @brief Removes the log ouput in terminal
 * 
 */
void Logger::SupressStdout() {
    _stdoutSupress = true;
}

/**
 * @brief Restores the log output in terminal
 * 
 */
void Logger::RestoreStdout() {
    _stdoutSupress = false;
}

/**
 * @brief Log at this specific level
 * 
 * @param s String to log
 */
void Logger::LogError(std::string s) {
    Log(ERROR, s);
}

/**
 * @brief Log at this specific level
 * 
 * @param s String to log
 */
void Logger::LogWarning(std::string s) {
    Log(WARNING, s);
}

/**
 * @brief Log at this specific level
 * 
 * @param s String to log
 */
void Logger::LogInfo(std::string s) {
    Log(INFO, s);
}

/**
 * @brief Log at this specific level
 * 
 * @param s String to log
 */
void Logger::LogDebug(std::string s) {
    Log(DEBUG, s);
}