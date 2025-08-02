#include "M5StickCPlusDisplayManager.h"
#include "SerialConfig.h"

M5StickCPlusDisplayManager::M5StickCPlusDisplayManager(M5Display& m5Display, int screenWidth, int maxLines)
    : display(m5Display)
    , baseStatusLine("")
    , scrollOffset(0)
    , maxLineWidth(screenWidth)
    , showingProgress(false)
    , progressCharCount(0)
    , maxDisplayLines(maxLines)
    , currentLineCount(0)
    , otaModeActive(false)
    , statusDisplayModeActive(false)
{
    displayLines = new String[maxDisplayLines];
}

M5StickCPlusDisplayManager::~M5StickCPlusDisplayManager() {
    delete[] displayLines;
}

void M5StickCPlusDisplayManager::startProgressAnimation() {
    // do nothing
}

void M5StickCPlusDisplayManager::stopProgressAnimation() {
    // do nothing
}

void M5StickCPlusDisplayManager::updateProgressAnimation(int yPosition) {
    // do nothing
}

void M5StickCPlusDisplayManager::addDisplayLine(const String& newLine, bool preserveWiFiLine, bool skipRefresh) {
    // do nothing
}

void M5StickCPlusDisplayManager::refreshDisplay() {
    // do nothing
}

void M5StickCPlusDisplayManager::clearDisplay() {
    // do nothing
}

void M5StickCPlusDisplayManager::setOTAMode(bool enabled) {
    otaModeActive = enabled;
}

void M5StickCPlusDisplayManager::setStatusDisplayMode(bool enabled) {
    statusDisplayModeActive = enabled;
}