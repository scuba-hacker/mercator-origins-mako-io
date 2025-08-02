#pragma once

#include <Arduino.h>
#include <M5StickCPlus.h>

class M5StickCPlusDisplayManager {
public:
    // Reference to the M5 display object
    M5Display& display;
    
    // Constructor
    M5StickCPlusDisplayManager(M5Display& m5Display, int screenWidth = 256, int maxLines = 4);
    
    // Destructor
    ~M5StickCPlusDisplayManager();
    
    // Multi-line scrolling display methods
    void addDisplayLine(const String& newLine, bool preserveWiFiLine = false, bool skipRefresh = false);
    void refreshDisplay();
    
    // Single-line scrolling status methods
    void updateScrollingStatusLine(const String& newText, bool append = true, bool scrollOffPrevious = false, int yPosition = 55);
    
    // Progress animation methods
    void startProgressAnimation();
    void stopProgressAnimation();
    void updateProgressAnimation(int yPosition = 55);
    
    void setStatusDisplayMode(bool enabled);
    bool isInStatusDisplayMode() const { return statusDisplayModeActive; }
    
    // Utility methods
    void clearDisplay();
    void setOTAMode(bool enabled);
    bool isInOTAMode() const { return otaModeActive; }
    bool isShowingProgress() const { return showingProgress; }
    int getCurrentLineCount() const { return currentLineCount; }
    String getCurrentScrollingStatusLine() const { return scrollingStatusLine; }

private:
    // Generic scrolling status line variables
    String scrollingStatusLine;
    String baseStatusLine;  // Base line without progress chars
    int scrollOffset;
    const int maxLineWidth;  // Full screen width
    bool showingProgress;
    int progressCharCount;
    
    // Display scrolling system variables
    const int maxDisplayLines;  // Max number of lines in multi-line display
    String* displayLines;
    int currentLineCount;
    
    // OTA mode flag
    bool otaModeActive;
    
    // Status display mode flag
    bool statusDisplayModeActive;
    
    // Private helper methods
    void updateScrollingStatusLineDisplay(int yPosition);
    void drawStatusIndicator(int x, int y, const String& label, bool status, const String& value = "");
};