#include "lidarConfig.h"

LidarConfig::LidarConfig(std::string _mode) {
    driver = *createLidarDriver();
    channel = *createSerialPortChannel(port, baudrate);
    modeName = _mode;
    if (modeName == "DenseBoost") modeId = 1;
}

LidarConfig::~LidarConfig() {}
