#include <thread>
#include <chrono>
#include <string>
#include "ArduinoDriver.h"


#define FW_VERSION "0.0.1"

namespace wheeltec_gripper_driver {


using namespace std;

void ArduinoDriver::init(std::string port, int baudrate)
{
    // @TODO read version from config
    version_ = FW_VERSION;

    // establish connection with teensy board
    boost::system::error_code ec;
    serial_port_.open(port, ec);

    if (ec)
    {
        ROS_WARN("Failed to connect to serial port %s", port.c_str());
        return;
    }
    else
    {
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(static_cast<uint32_t>(baudrate)));
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        ROS_INFO("Successfully connected to serial port %s", port.c_str());
        ROS_INFO("Wheeltec gripper is homing");
    }
    
    initialised_ = false;
    std::string msg = "0";

    ROS_INFO("Successfully initialised gripper driver on port %s", port.c_str());

}

ArduinoDriver::ArduinoDriver()
: serial_port_(io_service_)
{
}

// Update between hardware interface and hardware driver
void ArduinoDriver::update(double pos_commands)
{
    // construct update message
    std::string outMsg = degToSteps(pos_commands);

    // run the communication with board
    exchange(outMsg);

}

std::string ArduinoDriver::degToSteps(double deg)
{
    int steps = deg / degreesPerStep;
	return to_string(steps);
}

std::string ArduinoDriver::convertToString(char *a, int size)
{
     int i;
    string s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}

double ArduinoDriver::stepsToDegrees(int steps)
{
    return steps * degreesPerStep;
}

std::string convertToString(char* a, int size)
{
    int i;
    string s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}
// Send specific commands
void ArduinoDriver::sendCommand(std::string outMsg)
{
    exchange(outMsg);
}

// Send msg to board and collect data
void ArduinoDriver::exchange(std::string outMsg)
{
    std::string inMsg;
    std::string errTransmit = "";
    std::string errReceive = "";
    
    if (!transmit(outMsg, errTransmit))
    {
        // print err
    }

    if (!receive(inMsg, errReceive))
    {
        // print err
    }
    // parse msg

}

bool ArduinoDriver::transmit(std::string msg, std::string& err)
{
    boost::system::error_code ec;
    const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());

    boost::asio::write(serial_port_, sendBuffer, ec);
    ROS_WARN("Message Sent: %s", msg);
    if(!ec)
    {
        return true;
    }
    else
    {
        err = "Error in transmit";
        ROS_WARN("transmit error: %s", err);
        return false;
    }    
}

bool ArduinoDriver::receive(std::string& inMsg, std::string& err)
{
    char c;
    std::string msg = "";
    bool eol = false;
    while (!eol)
    {
        boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                eol = true;
            default:
                msg += c;
        }
    }
    inMsg = msg;
    return true;
}

void ArduinoDriver::checkInit(std::string msg)
{
    std::size_t ack_idx = msg.find("A", 2) + 1;
    std::size_t version_idx = msg.find("B", 2) + 1;
    int ack = std::stoi(msg.substr(ack_idx, version_idx));
    if (ack)
    {
        initialised_ = true;
    }
    else
    {
        std::string version = msg.substr(version_idx);
        ROS_ERROR("Firmware version mismatch %s", version);
    }  
}

 
} // namespace ar3_hardware_drivers
