#pragma once
#include <thread>

#include "carMasterBase.h"

//蜂鸣器控制器
class BuzzerController {
   public:
    BuzzerController(/* args */);
    ~BuzzerController();

    void startSound(serial::Serial& serial);

    void endSound(serial::Serial& serial);

   private:
};
