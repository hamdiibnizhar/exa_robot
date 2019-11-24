#include <ros/ros.h>
#include <ros/publisher.h>

#include <sys/socket.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/UserData.h>

#define USE_CLASS

inline int quality2dBm(int quality)
{
	// Quality to dBm:
	if(quality <= 0)
		return -100;
	else if(quality >= 100)
		return -50;
	else
		return (quality / 2) - 100;
}

#ifdef USE_CLASS
EmbeddedData geigerCounter; 
#elif
int dataGeiger = 0;
#endif

void GeigerCounterDataCallback(const std_msgs::String::Ptr& msg)
{
#ifdef USE_CLASS
	geigerCounter.data = msg->data;
#elif
	dataGeiger = msg->data;
#endif	
}

ros::Time stamp;
cv::Mat data(1, 2, CV_64FC1);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string interface = "wlan0";
	double rateHz = 0.5; // Hz
	std::string frameId = "base_link";

	pnh.param("interface", interface, interface);
	pnh.param("rate", rateHz, rateHz);
	pnh.param("frame_id", frameId, frameId);

	ros::Rate rate(rateHz);

	ros::Publisher wifiPub = nh.advertise<rtabmap_ros::UserData>("wifi_signal", 1);

	while(ros::ok())
	{
		ros::Subsciber emSub = nh.subscibe("GeigerMuller_CPM", 1000, GeigerCounterDataCallback)
#ifdef USE_CLASS
		if(geigerCounter.hasData())
#elif
		if(dataGeiger != 0)
#endif	
		{
#ifdef USE_CLASS
			geigerCounter.publish();
#elif
			stamp = ros::Time::now();
			// Create user data [level] with the value
			data.at<double>(0) = double(dataGeiger);

			// we should set stamp in data to be able to
			// retrieve it from rtabmap map data to get precise
			// position in the graph afterward
			data.at<double>(1) = stamp.toSec();

			rtabmap_ros::UserData dataMsg;
			dataMsg.header.frame_id = frameId;
			dataMsg.header.stamp = stamp;
			rtabmap_ros::userDataToROS(data, dataMsg, false);
			wifiPub.publish<rtabmap_ros::UserData>(dataMsg);
#endif
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

templete<typename T>
class EmbeddedData
{
private :
	cv::Mat data(1, 2, CV_64FC1);
	T data[_ndata];
	int _ndata;
	ros::Time stamp;


public :
	boolean hasData();
	GeigerDataReceiver()
	{
		_ndata = 0;
		ros::NodeHandle nh;
	}
	GeigerDataReceiver(ndata) ndata:_ndata
	{
		if(_ndata = 0) _ndata = 0;
		else _ndata--
		ros::NodeHandle nh;
	}
	void publish();
}

void EmbeddedData::publish()
{
	stamp = ros::Time::now();
	data.at<double>(0) = double(dataGeiger);
	data.at<double>(1) = stamp.toSec();

	rtabmap_ros::UserData dataMsg;
	dataMsg.header.frame_id = frameId;
	dataMsg.header.stamp = stamp;
	rtabmap_ros::userDataToROS(data, dataMsg, false);
	wifiPub.publish<rtabmap_ros::UserData>(dataMsg);
}

