#include <lucid_cam_driver/serial.hpp>


SerialComm::SerialComm(std::string busPath) {
    port = new SerialPort(busPath);
    port->SetBaudRate(BaudRate::BAUD_1152000);
    port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    port->SetParity(Parity::PARITY_NONE);
    port->SetStopBits(StopBits::STOP_BITS_1);
}

SerialComm::~SerialComm() {
    port->Close();
    delete port;
}

SerialResponse* SerialComm::SendDeviceCommand(DeviceCommands c) {
    DataBuffer request;
    DataBuffer response;
    request.push_back(c);
    port->Write(request);
    port->Read(response);
    try {
        return VecToSerialResponse(response);
    }
    catch (std::exception& e) {
        Logger::Log(LogLevel::ERROR, "exception caught in serial communication: %s", e.what());
        return nullptr;
    }

}

SerialResponse* SerialComm::SendCommand(Commands c, uint16_t arg) {
    DataBuffer request;
    DataBuffer response;
    request.push_back(c >> 8 & 0x00FF);
    request.push_back(c & 0x00FF);
    request.push_back(arg >> 8 & 0x00FF);
    request.push_back(arg & 0x00FF);

    port->Write(request);
    port->Read(response);
    try {
        return VecToSerialResponse(response);
    }
    catch (std::exception& e) {
        Logger::Log(LogLevel::ERROR, "exception caught in serial communication: %s", e.what());
        return nullptr;
    }
}

SerialResponse* SerialComm::VecToSerialResponse(const std::vector<uint8_t>& v) {
    if(v.size() != 9) throw std::runtime_error("Invalid response size");
    SerialResponse* s = new SerialResponse;
    uint8_t* repArray = (uint8_t*)s;
    for(uint8_t i = 0; i < 9; i++) {
        repArray[i] = v.data()[i];
    }
    return s;
}
