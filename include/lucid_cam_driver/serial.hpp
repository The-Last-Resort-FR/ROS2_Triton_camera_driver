#pragma once
#include <string>
#include <exception>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include <lucid_cam_driver/logger.hpp>

using namespace LibSerial;

enum DeviceCommands : uint8_t {
    PING = 0x00,
    STATUS = 0x01,

};

enum Commands : uint16_t  {
    GET_NEXT = 0x0000,
    SET_TRIG_FREQ = 0x0001,
    START_TRIG = 0x0002,
    STOP_TRIG = 0x0003,
};

enum Status : uint8_t {
    STATUS_READY = 0b00000000,
    STATUS_BUSY = 0b00000001,
    STATUS_DATA_NEXT = 0b00000010,
    STATUS_CMD_ERROR = 0b01000000,
    STATUS_ERROR = 0b10000000,
};

struct SerialResponse {
    uint8_t status;
    uint8_t data[8];
};


class SerialComm {
private:
    SerialPort* port;
public:
    SerialComm(std::string busPath);
    ~SerialComm();
    SerialResponse* SendDeviceCommand(DeviceCommands c);
    SerialResponse* SendCommand(Commands c, uint16_t arg = 0);
    static SerialResponse* VecToSerialResponse(const std::vector<uint8_t>& v);
};

