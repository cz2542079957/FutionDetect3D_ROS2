#include "buzzerController.h"

BuzzerController::BuzzerController(/* args */) {}

BuzzerController::~BuzzerController() {}

void BuzzerController::startSound(serial::Serial& serial) {
    std::vector<uint8_t> data = {0xff, 0xfc, 0x05, 0x02, 40, 0};
    CarMasterBase::send(serial, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    data = {0xff, 0xfc, 0x05, 0x02, 80, 0};
    CarMasterBase::send(serial, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    data = {0xff, 0xfc, 0x05, 0x02, 160, 0};
    CarMasterBase::send(serial, data);
}

void BuzzerController::endSound(serial::Serial& serial) {
    std::vector<uint8_t> data = {0xff, 0xfc, 0x05, 0x02, 160, 0};
    CarMasterBase::send(serial, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    data = {0xff, 0xfc, 0x05, 0x02, 80, 0};
    CarMasterBase::send(serial, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    data = {0xff, 0xfc, 0x05, 0x02, 40, 0};
    CarMasterBase::send(serial, data);
}
