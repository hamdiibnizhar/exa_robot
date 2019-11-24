#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d_transforms.h>

#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #define USE_CPS_ONLY

int radiation_warning_1 = 15,
    radiation_warning_2 = 30,
    radiation_warning_3 = 60;


inline int radiation_value(int counter)
{
	// dBm to Quality:
    if(counter > 0 && counter <= 5) //background
        return 0;
    else if(counter > 5 && counter <= radiation_warning_1)
    	return 2;
    else if(counter > radiation_warning_1 && counter <= radiation_warning_2)
        return 4;
    else if(counter > radiation_warning_2 && counter <= radiation_warning_3)
        return 5;
    else
    	return 10;
}

typedef     int     sDataPack;

#if defined (USE_CPS_ONLY)
    std::map<double, sDataPack> radiation_counter;
#else
    std::map<double, sDataPack, sDataPack> radiation_counter;
#endif

ros::Publisher geiger_data_cloud;
std::map<double, int> nodeStamps_;

void mapDataCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg)
{
	ROS_INFO("Received map data!");

	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	std::map<int, rtabmap::Signature> signatures;
	rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);

	for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
	{
		int id = iter->first;
		rtabmap::Signature & node = iter->second;

		nodeStamps_.insert(std::make_pair(node.getStamp(), node.id()));

		if(!node.sensorData().userDataCompressed().empty())
		{
			cv::Mat data;
			node.sensorData().uncompressDataConst(0 ,0, 0, &data);

			if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 2)
			{
				int geiger_counter = data.at<double>(0);
				double stamp = data.at<double>(1);
				radiation_counter.insert(std::make_pair(stamp, geiger_counter));
			}
			else if(!data.empty())
			{
				ROS_ERROR("Wrong user data format for geiger counter data.");
			}
		}
	}

	// for the logic below, we should keep only stamps for
	// nodes still in the graph (in case nodes are ignored when not moving)
	std::map<double, int> nodeStamps;
	for(std::map<double, int>::iterator iter=nodeStamps_.begin(); iter!=nodeStamps_.end(); ++iter)
	{
		std::map<int, rtabmap::Transform>::const_iterator jter = poses.find(iter->second);
		if(jter != poses.end())
		{
			nodeStamps.insert(*iter);
		}
	}

	if(radiation_counter.size() == 0)
	{
		ROS_WARN("No geiger counter data detected yet in user data of map data");
	}

	//============================
	// Add WIFI symbols
	//============================

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled_geiger_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	int id = 0;
	for(std::map<double, int>::iterator iter=radiation_counter.begin(); iter!=radiation_counter.end(); ++iter, ++id)
	{
		// The Wifi value may be taken between two nodes, interpolate its position.
		double stamp_geiger_counter = iter->first;
		std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stamp_geiger_counter); // lower bound of the stamp
		if(previousNode!=nodeStamps.end() && previousNode->first > stamp_geiger_counter && previousNode != nodeStamps.begin())
		{
			--previousNode;
		}
		std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(stamp_geiger_counter); // upper bound of the stamp

		if(previousNode != nodeStamps.end() &&
		   nextNode != nodeStamps.end() &&
		   previousNode->second != nextNode->second &&
		   uContains(poses, previousNode->second) && uContains(poses, nextNode->second))
		{
			rtabmap::Transform poseA = poses.at(previousNode->second);
			rtabmap::Transform poseB = poses.at(nextNode->second);
			double stampA = previousNode->first;
			double stampB = nextNode->first;
			UASSERT(stamp_geiger_counter>=stampA && stamp_geiger_counter <=stampB);

			rtabmap::Transform v = poseA.inverse() * poseB;
			double ratio = (stamp_geiger_counter-stampA)/(stampB-stampA);

			v.x()*=ratio;
			v.y()*=ratio;
			v.z()*=ratio;

			rtabmap::Transform geiger_counter_pose = (poseA*v).translation(); // rip off the rotation

			// Make a line with points
			int quality = radiation_value(iter->second);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			for(int i=0; i<10; ++i)
			{
				// 1 cm between each points
				// the number of points depends on the dBm (which varies from -30 (near) to -80 (far))
				pcl::PointXYZRGB pt;
				pt.z = float(i+1)*0.01f;
				// if(i<quality)
				// {
				// 	// green
				// 	pt.g = 255;
				// 	if(i<7)
				// 	{
				// 		// yellow
				// 		pt.r = 255;
				// 	}
				// }
				// else
				// {
				// 	// gray
				// 	pt.r = pt.g = pt.b = 100;
				// }

                switch(quality)
                {
                    case 0: pt.r = 0; pt.g = 255; pt.b = 0;
                        break;
                    case 2: pt.r = 64; pt.g = 255; pt.b = 0;
                        break;
                    case 4: pt.r = 128; pt.g = 255; pt.b = 0;
                        break;
                    case 5: pt.r = 225; pt.g = 0; pt.b = 0;
                        break;
                }
				cloud->push_back(pt);
			}
			pcl::PointXYZRGB anchor(255, 0, 0);
			cloud->push_back(anchor);

			cloud = rtabmap::util3d::transformPointCloud(cloud, geiger_counter_pose);

			if(assembled_geiger_data->size() == 0)
			{
				*assembled_geiger_data = *cloud;
			}
			else
			{
				*assembled_geiger_data += *cloud;
			}
		}
	}

	if(assembled_geiger_data->size())
	{
		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(*assembled_geiger_data, cloudMsg);
		cloudMsg.header = mapDataMsg->header;
		geiger_data_cloud.publish(cloudMsg);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "geiger_counter_data");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string interface = "geiger_counter";
	double rateHz = 0.5; // Hz
	std::string frameId = "base_link";

	geiger_data_cloud = nh.advertise<sensor_msgs::PointCloud2>("geiger_counter", 1);
	ros::Subscriber mapDataSub = nh.subscribe("/rtabmap/mapData", 1, mapDataCallback);

	ros::spin();

	return 0;
}