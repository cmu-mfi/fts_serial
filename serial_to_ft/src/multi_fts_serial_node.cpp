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

enum Sensor
{
    DESTROYER,
    ARCHITECT,
    BUILDER,
    CREATOR
};

class FTInterface
{
public:
    char *calfilepath;    // name of calibration file
    unsigned short index; // index of calibration in file (second parameter; default = 1)
    Calibration *cal;     // struct containing calibration information
    unsigned short i;     // loop variable used to print results
    short sts;            // return value from functions
    float *TT;

    FTInterface(std::string calfile, unsigned short index = 1)
    {
        char *calfilepath = const_cast<char *>(calfile.data());
        this->calfilepath = calfilepath;
        this->index = index;
        this->cal = createCalibration(calfilepath, index);
        if (cal == NULL)
        {
            std::cerr << "\nSpecified calibration could not be loaded: "<< calfile<< "\n";
	    std::cerr << calfilepath <<"\n";
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

    FTSensor(enum Sensor index, ros::NodeHandle nh) : fti(selectCalFile(index))
    {
        std::string pub_topic;
        std::string pub_raw_topic;
	std::stringstream ss1, ss2;

	switch(index){
		case ARCHITECT:
			ss1 << "yk_architect" << "/fts";
			ss2 << "yk_architect" << "/fts_raw";		
			break;
		case BUILDER:
			ss1 << "yk_builder" << "/fts";
			ss2 << "yk_builder" << "/fts_raw";		
			break;
		case CREATOR:
			ss1 << "yk_creator" << "/fts";
			ss2 << "yk_creator" << "/fts_raw";		
			break;
		case DESTROYER:
			ss1 << "yk_destroyer" << "/fts";
			ss2 << "yk_destroyer" << "/fts_raw";		
			break;

	}

        pub_topic = ss1.str();
        pub_raw_topic = ss2.str();
        
	pub = nh.advertise<geometry_msgs::WrenchStamped>(pub_topic, 1000);
        pub_raw = nh.advertise<geometry_msgs::WrenchStamped>(pub_raw_topic, 1000);
    }

private:
    static std::string selectCalFile(enum Sensor index)
    {
	std::string calfile;
        if (index == Sensor::ARCHITECT)
            calfile = "FT45384_r1.cal";
	else if (index == Sensor::BUILDER)
            calfile = "FT45383_r1.cal";
	else if (index == Sensor::CREATOR)
            calfile = "FT45386.cal";
	else if (index == Sensor::DESTROYER)
            calfile = "FT45385.cal";
        else
        {
            std::cerr << "Invalid sensor name";
            exit(1);
        }

        std::string calfilepath = "/home/mfi/repo/ros1_ws/src/fts_serial/serial_to_ft/config/" + calfile;
        char *file = const_cast<char *>(calfilepath.data());
	
        return calfilepath;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fts_serial_node");
    ros::NodeHandle nh;

    int baud_arg;
    ros::param::param<int>("~baudrate", baud_arg, 115200);
    ros::Rate loop_rate(500);

    std::string data;
    float voltages[6];
    float FT[6];
    geometry_msgs::WrenchStamped msg;
    geometry_msgs::WrenchStamped msg_raw;
    int index = 0;
    FTSensor fts[4] = {FTSensor((Sensor)index, nh), FTSensor((Sensor)++index, nh), FTSensor((Sensor)++index, nh), FTSensor((Sensor)++index, nh)};

    // read usb srt values
    SerialPortReader reader("/dev/ttyUSB0", baud_arg);

    std::cout<<"\nSetup done\n";

    while (ros::ok())
    {
        data = reader.readValues();
        std::stringstream ss(data);
    	enum Sensor device=static_cast<Sensor>(9);
        std::string temp;

        if (!data.empty())
        {
            std::stringstream ss(data);
            std::string temp;
            int id;

            // Expecting data in the form of "V: 1 0.00, 0.00, 0.00, 0.00, 0.00, 0.00"
            ss >> temp;
            while (temp != "V" && !ss.eof())
                ss >> temp;
            
	    if (temp == "V")
	    {
                //ss >> id >> voltages[0] >> temp >> voltages[1] >> temp >> voltages[2] >> temp >> voltages[3] >> temp >> voltages[4] >> temp >> voltages[5];
                ss >> id >> voltages[0] >> voltages[1] >> voltages[2] >> voltages[3] >> voltages[4] >> voltages[5];
                device = static_cast<Sensor>(id);
   	        //std::cout<<"I got values for "<<id<<std::endl;
	    }
        }

	if (device < 4 && device > -1)
	{
		//std::cout<<"\nGot Voltages from "<<device<<"\n";
		fts[device].fti.getFTvalues(FT, voltages);
		//std::cout<<"....";
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
		    msg.wrench.force.z -= 17.3;
		}

		fts[device].pub.publish(msg);
		fts[device].pub_raw.publish(msg_raw);

		//std::cout<<"\nPublished FTS for "<<device<<"\n";
	}
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
