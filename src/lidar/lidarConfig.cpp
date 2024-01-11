#include "lidarConfig.h"

LidarConfig::LidarConfig(std::string _mode ) {
    driver = *createLidarDriver();
    channel = *createSerialPortChannel(port, baudrate);
    mode = _mode;
}

LidarConfig::~LidarConfig() {}
