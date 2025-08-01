#pragma once

//#include <Arduino.h>

#ifdef USE_WEBSERIAL
  #include <WebSerial.h>
#endif

// Serial configuration macros and feature flags
// This header centralizes all serial/logging configuration to avoid duplication

// External declarations for global variables (defined in main.cpp)
extern bool writeLogToSerial;

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
#define USB_SERIAL_WRITE(x) do { if (writeLogToSerial) USB_SERIAL_BASE.write(x); } while(0)

// Non-conditional USB_SERIAL for direct access (like WebSerial setup)
#define USB_SERIAL USB_SERIAL_BASE