#include <iostream>
#include <string>
#include <vector>
#include <serial/serial.h>

class SerialPortReader
{
private:
    serial::Serial ser;

public:
    // Constructor
    SerialPortReader(const std::string &portName, const int baud_rate)
    {
        try
        {
            ser.setPort(portName);
            ser.setBaudrate(baud_rate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();

            if (ser.isOpen())
            {
                ROS_INFO_STREAM("Serial Port initialized");
            }
            else
            {
                exit(1);
            }
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port "<< portName);
            exit(1);
        }
    }

    // Method to read values
    std::string readValues()
    {
        if (ser.available())
        {
            std::string data;
            data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << data);

            return data;
        }
        else
        {
            ROS_ERROR_STREAM("No serial data available");
            return "";
        }
    }

    // Destructor
    ~SerialPortReader()
    {
    }
};