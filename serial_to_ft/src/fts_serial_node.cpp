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
    std::string ns = ros::this_node::getNamespace();
    // ros::param::get("~calfilepath", calfilepath_arg);
    // ros::param::get("~serialport", serial_arg);
    ros::param::param<int>("~baudrate", baud_arg, 115200);

    std::cout<<"namespace:"<<ns;

    ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("fts", 1000);
    ros::Publisher pub_raw = nh.advertise<geometry_msgs::WrenchStamped>("fts_raw", 1000);
    ros::Rate loop_rate(1000);

    geometry_msgs::WrenchStamped msg;
    geometry_msgs::WrenchStamped msg_raw;

    SerialPortReader reader("/dev/ttyUSB0", baud_arg);

    std::string data;
    float voltages[6];
    float FT[6];
    std::string calfile;

    if(ns.compare("/yk_builder")==0)
	calfile = "FT45383_r1.cal";
    else if(ns.compare("/yk_creator")==0)
	calfile = "FT45386.cal";
    else if(ns.compare("/yk_destroyer")==0)
	calfile = "FT45385.cal";
    else if(ns.compare("/yk_architect")==0)
	calfile = "FT45384_r1.cal";
 
    std::string calfilepath = "/home/mfi/repo/ros1_ws/src/fts_serial/serial_to_ft/config/" + calfile;
	    
    //char *calfilepath = "/home/mfi/repo/ros1_ws/src/fts_serial/serial_to_ft/config/FT45386.cal";
    // strcpy(calfilepath, calfilepath_arg.c_str());
    //FTInterface ft(calfilepath);
    char* file = const_cast<char *>(calfilepath.data());
    FTInterface ft(file);
    
    //Set Tool Transform
    std::string ttFilePath = "/home/mfi/repo/ros1_ws/fts_serial/serial_to_ft/config/TT.txt";
    std::ifstream ttFile(ttFilePath);
    float TT[6];
    for (int i=0; i<6; i++){
	    ttFile >> TT[i];
        TT[i] = 0.0;
    }
    ttFile.close();
    std::cout<<"Tool transform: "<<TT[0]<<" "<<TT[1]<<" "<<TT[2]<<" "<<TT[3]<<" "<<TT[4]<<" "<<TT[5]<<std::endl;
    std::cout << calfilepath << std::endl;
    ft.setToolTransform(TT);

    while (ros::ok())
    {
        data = reader.readValues();
        std::stringstream ss(data);
        std::string temp;

        if (!data.empty())
        {
            std::stringstream ss(data);
            std::string temp;

            // Expecting data in the form of "V: 0.00V, 0.00V, 0.00V, 0.00V, 0.00V, 0.00V"
            ss >> temp;
            while (temp != "V:" && !ss.eof())
            {
                ss >> temp;
                //ROS_INFO_STREAM("Still in here");
            }

            if (temp == "V:")
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


    	if(ns.compare("/yk_architect")==0){
	    msg.wrench.force.x += 1.5;
	    msg.wrench.force.y -= 6.5;	
	}

	pub.publish(msg);
        pub_raw.publish(msg_raw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
