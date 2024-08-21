#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include <fstream>
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
    float *TT;

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
    float temp_TT[] = {0,0,20,45,0,0};
    TT = temp_TT;
        sts = SetToolTransform(cal, TT, "mm", "degrees");
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

    void setToolTransform(float TT[])
    {
        sts = SetToolTransform(cal, TT, "mm", "degrees");
        switch (sts)
        {
        case 0:
            break; // successful completion
        case 1:
            std::cerr << "Invalid Calibration struct";
            exit(1);
        case 2:
            std::cerr << "Invalid distance units";
            exit(1);
        case 3:
            std::cerr << "Invalid angle units";
            exit(1);
        default:
            std::cerr << "Unknown error";
            exit(1);
        }
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

class FTSensor
{
public:
    FTInterface fti;
    ros::Publisher pub;
    ros::Publisher pub_raw;

    FTSensor(char *name, ros::NodeHandle nh) : fti(selectCalFile(name))
    {
        std::string pub_topic = std::format("{}/fts", name);
        std::string pub_raw_topic = std::format("{}/fts_raw", name);
        pub = nh.advertise<geometry_msgs::WrenchStamped>(pub_topic, 1000);
        pub_raw = nh.advertise<geometry_msgs::WrenchStamped>(pub_raw_topic, 1000);
    }

private:
    static char *selectCalFile(char *name)
    {
        char *calfile;
        if (strcmp(name, "yk_architect") == 0)
            calfile = "FT45384_r1.cal";
        else if (strcmp(name, "yk_builder") == 0)
            calfile = "FT45383_r1.cal";
        else if (strcmp(name, "yk_creator") == 0)
            calfile = "FT45386.cal";
        else if (strcmp(name, "yk_destroyer") == 0)
            calfile = "FT45385.cal";
        else
        {
            std::cerr << "Invalid sensor name";
            exit(1);
        }

        std::string calfilepath = "/home/mfi/repo/ros_ws/src/fts_serial/serial_to_ft/config/" + calfile;
        char *file = const_cast<char *>(calfilepath.data());
        return file;
    }
};

enum Sensor
{
    ARCHITECT,
    BUILDER,
    CREATOR,
    DESTROYER
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fts_serial_node");
    ros::NodeHandle nh;

    int baud_arg;
    ros::param::param<int>("~baudrate", baud_arg, 115200);
    ros::Rate loop_rate(1000);

    std::string data;
    enum Sensor device;
    float voltages[6];
    float FT[6];
    FTSensor fts[4] = {FTSensor("yk_architect", nh), FTSensor("yk_builder", nh), FTSensor("yk_creator", nh), FTSensor("yk_destroyer", nh)};

    // read usb srt values
    SerialPortReader reader("/dev/ttyUSB0", baud_arg);
    while (ros::ok())
    {
        data = reader.readValues();
        std::stringstream ss(data);
        std::string temp;

        if (!data.empty())
        {
            std::stringstream ss(data);
            std::string temp;
            int id;

            // Expecting data in the form of "V: 1 0.00, 0.00, 0.00, 0.00, 0.00, 0.00"
            ss >> temp;
            while (temp != "V:" && !ss.eof())
                ss >> temp;

            if (temp == "V:")
                ss >> id >> voltages[0] >> voltages[1] >> voltages[2] >> voltages[3] >> voltages[4] >> voltages[5];
                device = static_cast<Sensor>(id);
        }

        fts[device].fti.getFTvalues(FT, voltages);

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

        if (device == Sensor::ARCHITECT)
        {
            msg.wrench.force.x += 1.5;
            msg.wrench.force.y -= 6.5;
        }

        fts[device].pub.publish(msg);
        fts[device].pub_raw.publish(msg_raw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
