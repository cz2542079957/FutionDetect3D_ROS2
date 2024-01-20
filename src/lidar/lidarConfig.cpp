#include "lidarConfig.h"

LidarConfig::LidarConfig(std::string _port, std::string _mode) {
    driver = *createLidarDriver();
    port = _port;
    channel = *createSerialPortChannel(port, baudrate);
    modeName = _mode;
    if (modeName == "DenseBoost") modeId = 1;
}

LidarConfig::~LidarConfig() {}
