#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#ifdef USE_DATA_RECORD_TO_FILE
    #include <iostream>
    #include <fstream>
    #include <ctime>   
    #include <time.h>
    #include <string.h>
    #include <chrono>  
#endif

#ifdef USE_DATA_RECORD_TO_FILE
std::ofstream dataFile;

time_t theTime = time(NULL);
struct tm *aTime = localtime(&theTime);

int day = aTime->tm_mday;
int month = aTime->tm_mon + 1; // Month is 0 – 11, add 1 to get a jan-dec 1-12 concept
int year = aTime->tm_year + 1900; // Year is # years since 1900
int hour=aTime->tm_hour;
int minutes=aTime->tm_min;

//auto epoch = std::chrono::high_resolution_clock::from_time_t(0);


std::string filenameData = "geiger_receiver_" + std::to_string(year) + "_" + std::to_string(month) + "_" + std::to_string(day) + "_" + std::to_string(hour) + "_" + std::to_string(minutes) + ".txt";

#ifdef USE_CHRONO
	auto epoch = std::chrono::high_resolution_clock::now();
#else	
	std::clock_t start = std::clock();
#endif	
double timerun;

#endif

// typedef     std_msgs::Int64     sDataPack;
typedef     double     sDataPack;
//typedef     std_msgs::Float64     sDataPack;

struct geiger_pack
{
  sDataPack cps = 0;
  sDataPack cpm = 0;
};

geiger_pack geiger_sensor;

#ifdef USE_5_SECOND_TIMER_SAMPLE  
    double doseCoefUnderBase    = 0.000013747;  // in mSv , 5 second timer cpm
    double deadTimeGeiger       = 0.001500926;
#elif USE_15_SECOND_TIMER_SAMPLE 
    double doseCoefUnderBase    = 0.000013058;  // in mSv , 15 second timer cpm
    double deadTimeGeiger       = 0.0014084415;
#endif

//ros::Time stamp = ros::Time::now();
#ifdef USE_CPS  
void geiger_data_cps_callback(const std_msgs::Float64::ConstPtr& geiger_data_temp)
{
    geiger_sensor.cps = geiger_data_temp->data;
#ifdef USE_DATA_RECORD_TO_FILE
    dataFile << geiger_sensor.cps << "\t" << std::clock()/(double) CLOCKS_PER_SEC << "\n";
#endif
}
#endif
#ifdef USE_CPM
void geiger_data_cpm_callback(const std_msgs::Float64::ConstPtr& geiger_data_temp)
{
#ifdef USE_DATA_RECORD_TO_FILE
    //auto now   = std::chrono::high_resolution_clock::now();
#endif
#ifdef USE_DOSE_EQUIVALENT
    geiger_sensor.cpm = calculate_dose(double(geiger_data_temp->data);
#else
    geiger_sensor.cpm = double(geiger_data_temp->data);   
#endif
#ifdef USE_DATA_RECORD_TO_FILE
#ifdef USE_CHRONO
	auto now   = std::chrono::high_resolution_clock::now();
	timerun = std::chrono::duration_cast<std::chrono::microseconds>(now - epoch).count();
#else
	timerun = (std::clock()-start)/(double) CLOCKS_PER_SEC;
#endif			
    dataFile << geiger_sensor.cpm << "\t" << timerun << "\n";
    //dataFile << geiger_sensor.cpm << "\t" << std::chrono::duration_cast<std::chrono::microseconds>(now - epoch).count() << "\n";
#endif
#ifdef DEBUG
	ROS_INFO("masuk di cpm callback");
	ROS_INFO("geiger data callback %.6f", geiger_sensor.cpm);
#endif     
}
#endif

#ifdef USE_DOSE_EQUIVALENT
double calculate_dose(double countsPerTimes) //in mSv
{
    return countsPerTimes
#ifdef USE_CPS
    *60
#endif    
    / doseCoefUnderBase;
}
#endif

// cv::Mat data_collect(std_msgs::Int64 &data1, std_msgs::Int64 &data2)
// {
//     ros::Time stamp = ros::Time::now();
//     cv::Mat data(1, 3, CV_64FC1);
//     data.at<std_msgs::Int64>(0) = std_msgs::Int64(data1);
//     data.at<std_msgs::Int64>(1) = std_msgs::Int64(data2);
//     data.at<double>(2) = stamp.toSec();
//     return data;
// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "geiger_data_convert");

	ros::NodeHandle nh;                         
	ros::NodeHandle pnh("~");

#ifdef USE_DATA_RECORD_TO_FILE
    dataFile.open (filenameData, std::fstream::out);
#endif

	double rateHz = 0.5; // in Hz, to 0 to computation cost
	// std::string frameId = "geiger_counter_link";
    std::string frameId = "geiger_link";

	pnh.param("rate", rateHz, rateHz);
	pnh.param("frame_id", frameId, frameId);

    ros::Rate rate(rateHz);
	ros::Publisher  geigermap = nh.advertise<rtabmap_ros::UserData>("exa_robot/geiger_data", 100);
#ifdef USE_CPS  
    ros::Subscriber geiger_data_cps = nh.subscribe<std_msgs::Float64>("exa_robot/geiger_sensor_cps", 100, geiger_data_cps_callback);
#endif    
#ifdef USE_CPM
    ros::Subscriber geiger_data_cpm = nh.subscribe<std_msgs::Float64>("exa_robot/geiger_sensor_cpm", 100, geiger_data_cpm_callback);
#endif

    while(ros::ok())
    {
#ifdef DEBUG
		ROS_INFO("masuk di loop");
#endif
        ros::Time stamp = ros::Time::now();
#if defined (USE_CPS) && defined (USE_CPM) 
        cv::Mat data(1, 3, CV_64FC1);
#else 
        cv::Mat data(1, 2, CV_64FC1);        
#endif
        // data.at<std_msgs::Int64>(0) = std_msgs::Int64(geiger_sensor.cps);
        // data.at<std_msgs::Int64>(1) = std_msgs::Int64(geiger_sensor.cpm);
#ifdef USE_CPS        
        data.at<sDataPack>(0) = double(geiger_sensor.cps);
        data.at<double>(1) = stamp.toSec();
#endif
#ifdef USE_CPM
        data.at<sDataPack>(0) = double(geiger_sensor.cpm);
        data.at<double>(1) = stamp.toSec();
#endif
#if defined (USE_CPS) && defined (USE_CPM)
        data.at<sDataPack>(0) = geiger_sensor.cps;
        data.at<sDataPack>(1) = geiger_sensor.cpm;
        data.at<double>(2) = stamp.toSec();
#endif

        rtabmap_ros::UserData dataMsg;
        dataMsg.header.frame_id = frameId;
        dataMsg.header.stamp = stamp;

        rtabmap_ros::userDataToROS(data, dataMsg, false);

        geigermap.publish<rtabmap_ros::UserData>(dataMsg);
        ros::spinOnce();
        rate.sleep();
    }
#ifdef USE_DATA_RECORD_TO_FILE
    dataFile.close();
#endif    
    return 0;
}
