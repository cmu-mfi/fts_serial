#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "ftconfig.h"
#include "read_serial2.hpp"
#include <geometry_msgs/WrenchStamped.h>

class FTInterface
{
public:
    char *calfilepath;    // name of calibration file
    unsigned short index; // index of calibration in file (second parameter; default = 1)
    Calibration *cal;     // struct containing calibration information
    unsigned short i;     // loop variable used to print results
    short sts;            // return value from functions
    std::vector<float> TT;

    FTInterface(char *calfilepath, unsigned short index = 1)
    {
        this->calfilepath = calfilepath;
        this->index = index;
        this->cal = createCalibration(calfilepath, index);

        if (cal == NULL)
        {
            std::cerr << "\nSpecified calibration could not be loaded.\n";
            exit(1);
        }

        // Set force units.
        // This step is optional; by default, the units are inherited from the calibration file.
        sts = SetForceUnits(cal, "N");
        switch (sts)
        {
        case 0:
            break; // successful completion
        case 1:
            std::cerr << "Invalid Calibration struct";
            exit(1);
        case 2:
            std::cerr << "Invalid force units";
            exit(1);
        default:
            std::cerr << "Unknown error";
            exit(1);
        }

        // Set torque units.
        // This step is optional; by default, the units are inherited from the calibration file.
        sts = SetTorqueUnits(cal, "N-m");
        switch (sts)
        {
        case 0:
            break; // successful completion
        case 1:
            std::cerr << "Invalid Calibration struct";
            exit(1);
        case 2:
            std::cerr << "Invalid torque units";
            exit(1);
        default:
            std::cerr << "Unknown error";
            exit(1);
        }

        // Set tool transform.
        // This line is only required if you want to move or rotate the sensor's coordinate system.
        // This example tool transform translates the coordinate system 20 mm along the Z-axis
        // and rotates it 45 degrees about the X-axis.
        /*
        TT = {0, 0, 20, 45, 0, 0};
        sts = SetToolTransform(cal, SampleTT, "mm", "degrees");
        switch (sts)
        {
        case 0:
            break; // successful completion
        case 1:
            std::cerr<<"Invalid Calibration struct";
            exit(1);
        case 2:
            std::cerr<<"Invalid distance units";
            exit(1);
        case 3:
            std::cerr<<"Invalid angle units";
            exit(1);
        default:
            std::cerr<<"Unknown error";
            exit(1);
        }
        */

        // Temperature compensation is on by default if it is available.
        // To explicitly disable temperature compensation, uncomment the following code
        /*SetTempComp(cal,FALSE);                   // disable temperature compensation
        switch (sts) {
            case 0: break;	// successful completion
            case 1: printf("Invalid Calibration struct"); return 0;
            case 2: printf("Temperature Compensation not available on this transducer"); return 0;
            default: printf("Unknown error"); return 0;
        }*/

        /*
        float SampleBias[7] = {0.2651, -0.0177, -0.0384, -0.0427, -0.1891, 0.1373, -3.2423};

        // store an unloaded measurement; this removes the effect of tooling weight
        Bias(cal, SampleBias);
        */
    }

    void getFTvalues(float *FT, float *readings)
    {
        // convert a loaded measurement into forces and torques
        ConvertToFT(cal, readings, FT);
    }

    ~FTInterface()
    {
        destroyCalibration(cal);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fts_serial_node");
    ros::NodeHandle nh;

    std::string calfilepath_arg;
    std::string serial_arg;
    int baud_arg;
    // ros::param::get("~calfilepath", calfilepath_arg);
    // ros::param::get("~serialport", serial_arg);
    ros::param::param<int>("~baudrate", baud_arg, 115200);

    ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("fts", 1000);
    ros::Publisher pub_raw = nh.advertise<geometry_msgs::WrenchStamped>("fts_raw", 1000);
    ros::Rate loop_rate(1000);

    geometry_msgs::WrenchStamped msg;
    geometry_msgs::WrenchStamped msg_raw;

    SerialPortReader reader("/dev/ttyUSB0", baud_arg);

    std::string data;
    float voltages[6];
    float FT[6];

    char *calfilepath = "/home/mfi/repos/ros1_ws/src/fts_serial/serial_to_ft/config/FT45383.cal";
    // strcpy(calfilepath, calfilepath_arg.c_str());
    FTInterface ft(calfilepath);

    while (ros::ok())
    {
        data = reader.readValues();
        std::stringstream ss(data);
        std::string temp;

        if (!data.empty())
        {
            std::stringstream ss(data);
            std::string temp;

            // Expecting data in the form of "Voltages: 0.00V, 0.00V, 0.00V, 0.00V, 0.00V, 0.00V"
            ss >> temp;
            while (temp != "Voltages:" && !ss.eof())
            {
                ss >> temp;
                //ROS_INFO_STREAM("Still in here");
            }

            if (temp == "Voltages:")
                ss >> voltages[0] >> temp >> voltages[1] >> temp >> voltages[2] >> temp >> voltages[3] >> temp >> voltages[4] >> temp >> voltages[5];
        }

        ft.getFTvalues(FT, voltages);

        msg.header.stamp = ros::Time::now();
        msg.wrench.force.x = FT[0];
        msg.wrench.force.y = FT[1];
        msg.wrench.force.z = FT[2];
        msg.wrench.torque.x = FT[3];
        msg.wrench.torque.y = FT[4];
        msg.wrench.torque.z = FT[5];

        msg_raw.header.stamp = ros::Time::now();
        msg_raw.wrench.force.x = voltages[0];
        msg_raw.wrench.force.y = voltages[1];
        msg_raw.wrench.force.z = voltages[2];
        msg_raw.wrench.torque.x = voltages[3];
        msg_raw.wrench.torque.y = voltages[4];
        msg_raw.wrench.torque.z = voltages[5];

        pub.publish(msg);
        pub_raw.publish(msg_raw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
