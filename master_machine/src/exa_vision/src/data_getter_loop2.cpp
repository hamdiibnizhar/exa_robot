#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <fstream>
#include <ctime>   
#include <time.h>
#include <string.h>
#include <chrono> 

#define POSE_X  "Position X"
#define POSE_Y  "Position Y"
#define POSE_Z  "Position Z"
#define TWIST_X  "Speed X"
#define TWIST_Y  "Speed Y"
#define TWIST_Z  "Speed Z"
#define GEIGER_COUNTER  "CPM"
#define DISTANCE_STEP  "Distance Step"
#define TIME_STEP  "Time Step"
#define TIME_NOW  "Time Now"

#ifdef USE_CSV_FORMAT
    #define SEPARATOR   ","
    #define EXTENSION   ".csv"
#else
     #define SEPARATOR   "\t"
     #define EXTENSION   ".txt"
#endif


std::ofstream dataFile;

time_t theTime = time(NULL);
struct tm *aTime = localtime(&theTime);

int day = aTime->tm_mday;
int month = aTime->tm_mon + 1; // Month is 0 – 11, add 1 to get a jan-dec 1-12 concept
int year = aTime->tm_year + 1900; // Year is # years since 1900
int hour=aTime->tm_hour;
int minutes=aTime->tm_min;

//auto epoch = std::chrono::high_resolution_clock::from_time_t(0);


std::string filenameData = "exa_robot_loop2_" + std::to_string(year) + "_" + std::to_string(month) + "_" + std::to_string(day) + "_" + std::to_string(hour) + "_" + std::to_string(minutes) + EXTENSION;

bool isDataEmpty = false;
double distance = 0;
long double distanceTemp = 0;
//rtabmap::Transform poseNow(0,0,0,0,0,0);
//rtabmap::Transform posePrevious(0,0,0,0,0,0);

float xNow = 0, yNow=0, zNow=0, xPrev=0, yPrev=0, zPrev=0, xTwist=0, yTwist=0, zTwist=0;

double geigerData = 0;
bool isSave = false;

#ifdef USE_ROS_TIME
ros::Time start;
ros::Duration timerun;
#else
double timerun;
std::clock_t start;
#endif

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
double xTemp = msg->pose.pose.position.x;
double yTemp = msg->pose.pose.position.y;
double zTemp = msg->pose.pose.position.z;

    xNow = msg->pose.pose.position.x;
    yNow = msg->pose.pose.position.y;
    zNow = msg->pose.pose.position.z;

    xTwist = msg->twist.twist.linear.x;
    yTwist = msg->twist.twist.linear.y;
    zTwist = msg->twist.twist.linear.z;

    xTemp-=xPrev;
	yTemp-=yPrev;
    zTemp-=zPrev;

	xTemp*=xTemp;
	yTemp*=yTemp;
	zTemp*=zTemp;

	distance = sqrt(xTemp +  yTemp + zTemp);
	distanceTemp += distance;

    xPrev = msg->pose.pose.position.x;
    yPrev = msg->pose.pose.position.y;
    zPrev = msg->pose.pose.position.z;
    ROS_INFO("masuk odom");
}

void geiger_data_cpm_callback(const std_msgs::Float64::ConstPtr& geiger_data_temp)
{
#ifdef USE_CHRONO
	auto now   = std::chrono::high_resolution_clock::now();
	timerun = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
#elif USE_ROS_TIME
    timerun = ros::Time::now()-start;
#else
	timerun = (std::clock()-start)/(double) CLOCKS_PER_SEC;
#endif
    geigerData = geiger_data_temp->data;
    isSave = true;
    ROS_INFO("masuk geiger");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "saving_data_robot_loop2");
    dataFile.open (filenameData, std::fstream::out);
    ros::NodeHandle n;
#ifdef USE_CHRONO
    auto start = std::chrono::high_resolution_clock::now();
#elif USE_ROS_TIME
    start = ros::Time::now();
#else
    start = std::clock();
#endif
    dataFile << POSE_X << SEPARATOR << POSE_Y << SEPARATOR << POSE_Z << SEPARATOR <<  TWIST_X << SEPARATOR << TWIST_Y << SEPARATOR << TWIST_Z << SEPARATOR << GEIGER_COUNTER  << SEPARATOR << DISTANCE_STEP << SEPARATOR << TIME_STEP  << SEPARATOR << TIME_NOW << SEPARATOR << SEPARATOR << "Time Begin" << SEPARATOR << ros::Time::now() << "\n";
    ros::Subscriber sub = n.subscribe("/rtabmap/odom", 1000, chatterCallback);
    ros::Subscriber geiger_data_cpm = n.subscribe<std_msgs::Float64>("/exa_robot/geiger_sensor_cpm", 100, geiger_data_cpm_callback);
    while (ros::ok())
    {
        if(isSave)
        {
            ROS_INFO("masuk if");
            dataFile << xNow << SEPARATOR << yNow << SEPARATOR << zNow << SEPARATOR <<  xTwist << SEPARATOR << yTwist << SEPARATOR << zTwist << SEPARATOR << geigerData  << SEPARATOR << distanceTemp << SEPARATOR << timerun  << SEPARATOR << ros::Time::now() << "\n";
        }
        isSave = false;
        ros::spinOnce();
    }
    dataFile.close();
}