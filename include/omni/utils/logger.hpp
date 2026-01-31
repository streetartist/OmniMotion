/**
 * @file logger.hpp
 * @brief Logging utilities for debugging and diagnostics
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <functional>

namespace omni::utils {

/**
 * @brief Log levels
 */
enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Fatal,
    Off
};

/**
 * @brief Logger class for motor control diagnostics
 */
class Logger {
public:
    /**
     * @brief Log output callback type
     */
    using OutputCallback = std::function<void(LogLevel, const char*)>;

    /**
     * @brief Get singleton instance
     */
    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    /**
     * @brief Set minimum log level
     * @param level Minimum level to output
     */
    void setLevel(LogLevel level) {
        minLevel_ = level;
    }

    /**
     * @brief Get current log level
     */
    LogLevel getLevel() const { return minLevel_; }

    /**
     * @brief Set output callback
     * @param callback Function to handle log output
     */
    void setOutputCallback(OutputCallback callback) {
        outputCallback_ = callback;
    }

    /**
     * @brief Enable/disable timestamps
     */
    void enableTimestamps(bool enable) {
        timestamps_ = enable;
    }

    /**
     * @brief Log message at specified level
     */
    void log(LogLevel level, const char* format, ...) {
        if (level < minLevel_) return;

        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        if (outputCallback_) {
            outputCallback_(level, buffer);
        } else {
            // Default output to stderr
            const char* levelStr = getLevelString(level);
            std::fprintf(stderr, "[%s] %s\n", levelStr, buffer);
        }
    }

    // Convenience methods
    void trace(const char* format, ...) {
        if (LogLevel::Trace < minLevel_) return;
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LogLevel::Trace, "%s", buffer);
    }

    void debug(const char* format, ...) {
        if (LogLevel::Debug < minLevel_) return;
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LogLevel::Debug, "%s", buffer);
    }

    void info(const char* format, ...) {
        if (LogLevel::Info < minLevel_) return;
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LogLevel::Info, "%s", buffer);
    }

    void warning(const char* format, ...) {
        if (LogLevel::Warning < minLevel_) return;
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LogLevel::Warning, "%s", buffer);
    }

    void error(const char* format, ...) {
        if (LogLevel::Error < minLevel_) return;
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LogLevel::Error, "%s", buffer);
    }

    void fatal(const char* format, ...) {
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        log(LogLevel::Fatal, "%s", buffer);
    }

private:
    Logger() : minLevel_(LogLevel::Info), timestamps_(false) {}

    LogLevel minLevel_;
    OutputCallback outputCallback_;
    bool timestamps_;

    static const char* getLevelString(LogLevel level) {
        switch (level) {
            case LogLevel::Trace:   return "TRACE";
            case LogLevel::Debug:   return "DEBUG";
            case LogLevel::Info:    return "INFO ";
            case LogLevel::Warning: return "WARN ";
            case LogLevel::Error:   return "ERROR";
            case LogLevel::Fatal:   return "FATAL";
            default:                return "?????";
        }
    }
};

// Global logging macros
#define OMNI_LOG_TRACE(...)   omni::utils::Logger::instance().trace(__VA_ARGS__)
#define OMNI_LOG_DEBUG(...)   omni::utils::Logger::instance().debug(__VA_ARGS__)
#define OMNI_LOG_INFO(...)    omni::utils::Logger::instance().info(__VA_ARGS__)
#define OMNI_LOG_WARNING(...) omni::utils::Logger::instance().warning(__VA_ARGS__)
#define OMNI_LOG_ERROR(...)   omni::utils::Logger::instance().error(__VA_ARGS__)
#define OMNI_LOG_FATAL(...)   omni::utils::Logger::instance().fatal(__VA_ARGS__)

/**
 * @brief Data recorder for logging motor data over time
 */
template<typename T, size_t MaxSamples = 1000>
class DataRecorder {
public:
    struct Sample {
        float timestamp;
        T data;
    };

    DataRecorder() : count_(0), recording_(false), startTime_(0) {}

    /**
     * @brief Start recording
     */
    void start() {
        count_ = 0;
        recording_ = true;
        startTime_ = 0;  // Will be set on first sample
    }

    /**
     * @brief Stop recording
     */
    void stop() {
        recording_ = false;
    }

    /**
     * @brief Record a sample
     * @param timestamp Current time
     * @param data Data to record
     * @return true if recorded
     */
    bool record(float timestamp, const T& data) {
        if (!recording_ || count_ >= MaxSamples) {
            return false;
        }

        if (count_ == 0) {
            startTime_ = timestamp;
        }

        samples_[count_].timestamp = timestamp - startTime_;
        samples_[count_].data = data;
        count_++;
        return true;
    }

    /**
     * @brief Check if recording
     */
    bool isRecording() const { return recording_; }

    /**
     * @brief Check if buffer is full
     */
    bool isFull() const { return count_ >= MaxSamples; }

    /**
     * @brief Get sample count
     */
    size_t getCount() const { return count_; }

    /**
     * @brief Get sample by index
     */
    const Sample& getSample(size_t index) const {
        return samples_[index];
    }

    /**
     * @brief Clear all samples
     */
    void clear() {
        count_ = 0;
        recording_ = false;
    }

    /**
     * @brief Get recording duration
     */
    float getDuration() const {
        if (count_ == 0) return 0;
        return samples_[count_ - 1].timestamp;
    }

private:
    Sample samples_[MaxSamples];
    size_t count_;
    bool recording_;
    float startTime_;
};

}  // namespace omni::utils
