#pragma once

#include <Arduino.h>
//#include <WebSerial.h>

// Serial configuration macros and feature flags
// This header centralizes all serial/logging configuration to avoid duplication

// External declarations for global variables (defined in main.cpp)
extern bool writeLogToSerial;
extern bool writeTelemetryLogToSerial;

//#ifndef USE_WEBSERIAL        // make sure this is disabled if writeLogToSerial is false - Uncomment to enable
//  #define USE_WEBSERIAL   // make sure this is disabled if writeLogToSerial is false - Uncomment to enable
//#endif                   // make sure this is disabled if writeLogToSerial is false - Uncomment to enable

// Serial base configuration
#ifdef USE_WEBSERIAL
  #define USB_SERIAL_BASE WebSerial
#else
  #define USB_SERIAL_BASE Serial
#endif

// Conditional serial macros that respect writeLogToSerial flag
#define USB_SERIAL_PRINTF(...) do { if (writeLogToSerial) USB_SERIAL_BASE.printf(__VA_ARGS__); } while(0)
#define USB_SERIAL_PRINTLN(...) do { if (writeLogToSerial) USB_SERIAL_BASE.println(__VA_ARGS__); } while(0)
#define USB_SERIAL_PRINT(...) do { if (writeLogToSerial) USB_SERIAL_BASE.print(__VA_ARGS__); } while(0)

// Non-conditional USB_SERIAL for direct access (like WebSerial setup)
#define USB_SERIAL USB_SERIAL_BASE

// ============================================================================
// BUFFER_LOG - Static buffer logging system
// ============================================================================

// Configuration - adjust buffer size as needed
#ifndef BUFFER_LOG_SIZE
#define BUFFER_LOG_SIZE 1024  // 1 KB default buffer size
#endif

// Static buffer logging implementation
class BufferLogger {
private:
    static char buffer[BUFFER_LOG_SIZE];
    static size_t currentLength;

public:
    // Reset buffer to empty
    static void reset() {
        currentLength = 0;
        if (BUFFER_LOG_SIZE > 0) {
            buffer[0] = '\0';
        }
    }

    // Get current buffer contents
    static const char* getBuffer() {
        return buffer;
    }

    // Get current buffer length
    static size_t getLength() {
        return currentLength;
    }

    // Get available space
    static size_t getAvailableSpace() {
        return (currentLength < BUFFER_LOG_SIZE - 1) ? (BUFFER_LOG_SIZE - 1 - currentLength) : 0;
    }

    // Printf-style logging
    static void printf(const char* format, ...) {
        if (currentLength >= BUFFER_LOG_SIZE - 1) return;  // Buffer full

        va_list args;
        va_start(args, format);

        size_t remainingSpace = BUFFER_LOG_SIZE - 1 - currentLength;
        int written = vsnprintf(&buffer[currentLength], remainingSpace, format, args);

        va_end(args);

        if (written > 0 && written < (int)remainingSpace) {
            currentLength += written;
        } else if (written > 0) {
            // Truncated - fill to end of buffer
            currentLength = BUFFER_LOG_SIZE - 1;
            buffer[currentLength] = '\0';
        }
    }

    // Print string
    static void print(const char* str) {
        printf("%s", str);
    }

    // Print string with newline
    static void println(const char* str = "") {
        printf("%s\n", str);
    }

    // Print with various types (for compatibility)
    static void print(int value) { printf("%d", value); }
    static void print(unsigned int value) { printf("%u", value); }
    static void print(long value) { printf("%ld", value); }
    static void print(unsigned long value) { printf("%lu", value); }
    static void print(float value) { printf("%.2f", value); }
    static void print(double value) { printf("%.6f", value); }

    static void println(int value) { printf("%d\n", value); }
    static void println(unsigned int value) { printf("%u\n", value); }
    static void println(long value) { printf("%ld\n", value); }
    static void println(unsigned long value) { printf("%lu\n", value); }
    static void println(float value) { printf("%.2f\n", value); }
    static void println(double value) { printf("%.6f\n", value); }

    // Flush buffer to USB_SERIAL (when available)
    static void flushToSerial() {
        if (currentLength > 0) {
            USB_SERIAL.print(buffer);
            reset();  // Clear buffer after flushing
        }
    }
};

// Static member definitions (must be in header for inline class)
inline char BufferLogger::buffer[BUFFER_LOG_SIZE];
inline size_t BufferLogger::currentLength = 0;

// BUFFER_LOG macros - always log regardless of writeLogToSerial flag
#define BUFFER_LOG_PRINTF(...) BufferLogger::printf(__VA_ARGS__)
#define BUFFER_LOG_PRINTLN(...) BufferLogger::println(__VA_ARGS__)
#define BUFFER_LOG_PRINT(...) BufferLogger::print(__VA_ARGS__)
#define BUFFER_LOG_RESET() BufferLogger::reset()
#define BUFFER_LOG_GET_BUFFER() BufferLogger::getBuffer()
#define BUFFER_LOG_GET_LENGTH() BufferLogger::getLength()
#define BUFFER_LOG_FLUSH_TO_SERIAL() BufferLogger::flushToSerial()