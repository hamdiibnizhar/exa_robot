#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"


// typedef     std_msgs::Int64     sDataPack;
typedef     int     sDataPack;

struct geiger_pack
{
  sDataPack cps = 0;
  sDataPack cpm = 0;
};

geiger_pack geiger_sensor;

ros::Time stamp = ros::Time::now();

void geiger_data_cps_callback(const std_msgs::Int64::ConstPtr& geiger_data_temp)
{
    geiger_sensor.cps = geiger_data_temp->data;
    // geiger_sensor.cpm = geiger_data_temp.cpm.data;
}

#if !defined (USE_CPS_ONLY)
void geiger_data_cpm_callback(const std_msgs::Int64::ConstPtr& geiger_data_temp)
{
    // geiger_sensor.cps = geiger_data_temp.cps.data;
    geiger_sensor.cpm = geiger_data_temp->data;
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
   
    ros::Rate rate(0.5); 
    ros::Subscriber geiger_data_cps = nh.subscribe<std_msgs::Int64>("exa_robot/geiger_sensor_cps", 1, geiger_data_cps_callback);
#if !defined (USE_CPS_ONLY)
    ros::Subscriber geiger_data_cpm = nh.subscribe<std_msgs::Int64>("exa_robot/geiger_sensor_cpm", 1, geiger_data_cpm_callback);
#endif
    ros::Publisher  geigermap = nh.advertise<rtabmap_ros::UserData>("exa_robot/detector_data_map", 1);

    while(ros::ok())
    {
        ros::Time stamp = ros::Time::now();
#if defined (USE_CPS_ONLY)
        cv::Mat data(1, 2, CV_64FC1);
#else
        cv::Mat data(1, 3, CV_64FC1);
#endif
        // data.at<std_msgs::Int64>(0) = std_msgs::Int64(geiger_sensor.cps);
        // data.at<std_msgs::Int64>(1) = std_msgs::Int64(geiger_sensor.cpm);
        data.at<sDataPack>(0) = geiger_sensor.cps;
#if defined (USE_CPS_ONLY)
        data.at<double>(1) = stamp.toSec();
#else        
        data.at<sDataPack>(1) = geiger_sensor.cpm;
        data.at<double>(2) = stamp.toSec();
#endif

        rtabmap_ros::UserData dataMsg;
        dataMsg.header.frame_id = "geiger_data";
        dataMsg.header.stamp = stamp;

        rtabmap_ros::userDataToROS(data, dataMsg, false);

        geigermap.publish<rtabmap_ros::UserData>(dataMsg);
        ros::spin();
        rate.sleep();
    }
    return 0;
}