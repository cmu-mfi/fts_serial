#include <iostream>
#include <string>
#include <vector>
#include <libserialport.h>

class SerialPortReader
{
private:
    sp_port *port;
    std::string port_name;
    const size_t buf_size = 1024;
    std::vector<char> buf;

public:
    // Constructor
    SerialPortReader(const std::string &portName, const int baud_rate) : port_name(portName), buf(buf_size)
    {
        // Get the port by name
        if (sp_get_port_by_name(port_name.c_str(), &port) != SP_OK)
        {
            std::cerr << "Error finding the port " << port_name << std::endl;
            exit(1);
        }

        // Open the port
        if (sp_open(port, SP_MODE_READ) != SP_OK)
        {
            std::cerr << "Error opening the port " << port_name << std::endl;
            exit(1);
        }

        // Set the baud rate
        if (sp_set_baudrate(port, baud_rate) != SP_OK)
        {
            std::cerr << "Error setting the baud rate" << std::endl;
            exit(1);
        }

        // Set other serial parameters (8N1 configuration)
        sp_set_bits(port, 8);
        sp_set_parity(port, SP_PARITY_NONE);
        sp_set_stopbits(port, 1);
        sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE);
    }

    // Method to read values
    std::string readValues()
    {
        int bytes_read = sp_blocking_read(port, buf.data(), buf_size - 1, 1000);
        if (bytes_read > 0)
        {
            buf[bytes_read] = '\0'; // Null-terminate the string
            return std::string(buf.data());
        }
        else
        {
            std::cerr << "Error reading from port or no data received." << std::endl;
            return "";
        }
    }

    // Destructor
    ~SerialPortReader()
    {
        sp_close(port);
        sp_free_port(port);
    }
};

// Example usage
/*
int main()
{
    SerialPortReader reader("COM3");
    while (true)
    {
        std::string data = reader.readValues();
        if (!data.empty())
        {
            std::cout << "Received: " << data << std::endl;
        }
    }
    return 0;
}
*/