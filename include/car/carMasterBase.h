#pragma once
#include <algorithm>
#include <numeric>
#include <vector>

#include "serial.h"

class CarMasterBase {
   public:
    static void send(serial::Serial& serial, std::vector<uint8_t> dataWithoutCheckSum) {
        if (serial.isOpen()) {
            // 计算校验和
            uint8_t checkSum = std::accumulate(dataWithoutCheckSum.begin() + 2, dataWithoutCheckSum.end(), 0);
            checkSum = checkSum % 256;
            // 将校验和添加到数据包的末尾
            dataWithoutCheckSum.push_back(checkSum);
            // 发送包含校验和的数据包
            serial.write(dataWithoutCheckSum);
        }
    }
};
