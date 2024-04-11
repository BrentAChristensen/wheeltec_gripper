#ifndef ARDUINO_DRIVER_H
#define ARDUINO_DRIVER_H

#include <vector>
#include <unordered_map>
#include <string>
#include "math.h"
#include "time.h"
#include <ros/ros.h>
#include <boost/asio.hpp>

namespace wheeltec_gripper_driver 
{
  class ArduinoDriver 
  {
    public:
      const double degreesPerStep = 0.000340909090909091;
      void init(std::string port, int baudrate);
      void update(double pos_commands);
      std::string degToSteps(double deg);
      std::string convertToString(char* a, int size);
      double stepsToDegrees(int steps);
    
      ArduinoDriver();

    private:
      bool initialised_;
      std::string version_;
      boost::asio::io_service io_service_;
      boost::asio::serial_port serial_port_;
      std::vector<int> enc_commands_;
      std::vector<int> enc_steps_;
      std::vector<double> enc_steps_per_deg_;
      std::vector<int> enc_calibrations_;
      // Comms with Arduino
      void exchange(std::string outMsg); // exchange joint commands/state
      bool transmit(std::string outMsg, std::string& err);
      bool receive(std::string &inMsg, std::string& err);
      void sendCommand(std::string outMsg); // send arbitrary commands
      void checkInit(std::string msg);
      
 
  };
} // namespace wheeltec_hardware_drivers

#endif // Arduino driver
