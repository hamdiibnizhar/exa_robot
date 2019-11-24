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

#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// using namespace std::chrono_literals;

#ifdef USE_DATA_RECORD_TO_FILE
    //#include <iostream>
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


std::string filenameData = "geiger_map_" + std::to_string(year) + "_" + std::to_string(month) + "_" + std::to_string(day) + "_" + std::to_string(hour) + "_" + std::to_string(minutes) + ".txt";

bool isDataEmpty = false;
double distance = 0;
long double distanceTemp = 0;
//rtabmap::Transform poseNow(0,0,0,0,0,0);
//rtabmap::Transform posePrevious(0,0,0,0,0,0);

float xNow = 0, yNow=0, zNow=0, xPrev=0, yPrev=0, zPrev=0;
#ifdef USE_CHRONO
	auto epoch = std::chrono::high_resolution_clock::now();
#else	
	std::clock_t start = std::clock();
#endif	
double timerun;

#endif


#if defined (USE_DOSE_EQUIVALENT) && (defined (USE_CPM) || defined (USE_CPM))
double 	background_radiation 	= 0.0167,
		radiation_warning_1 	= 0.01,
    	radiation_warning_2 	= 0.5,
    	radiation_warning_3 	= 0.8;
#elif defined (USE_CPS)
double 	background_radiation 	= 0.8,
		radiation_warning_1 	= 100,
    	radiation_warning_2 	= 200,
    	radiation_warning_3 	= 400;
#elif defined (USE_CPM) // =background+150+50*(n-1)
double background_radiation 	= 50,
		radiation_warning_1 	= 200,
    	radiation_warning_2 	= 400,
    	radiation_warning_3 	= 650,
		radiation_warning_4 	= 950,
		radiation_warning_5 	= 1300,
		radiation_warning_6 	= 1700,
		radiation_warning_7 	= 2150,
		radiation_warning_8 	= 2650,
		radiation_warning_9 	= 3200;
int 	max_value_range = 3000, 
		max_mapped_value = 10;
double 	tolerance_value = background_radiation * max_mapped_value / max_value_range;
#endif

double mapValue(double y, double min_x, double max_x, double min_y, double max_y)
{
	return (min_x + (((y-min_y)/(max_y-min_y))*(max_x-min_x)));
}

//invers square law
float intensityByDistance(float source_intensity, float distance) //its distance 3 cm from detector
{
	return (source_intensity * 0.0009 /(distance*distance));
}

#ifdef USE_COLOR_MAP

template<class T>
class ColorMap
{
	T MapValue(T y, T min_x, T max_x, T min_y, T max_y);
public:
	ColorMap();
	~ColorMap();
	int r,g,b;
	void valueToColorMap(T val, T minValue, T maxValue, T tolerance);
	void valueToColorMap(T val, T minValue, T maxValue);
};

template <class T>
ColorMap<T>::ColorMap()
{
}

template <class T>
ColorMap<T>::~ColorMap()
{
}

template <class T>
inline T ColorMap<T>::MapValue(T y, T min_x, T max_x, T min_y, T max_y)
{
	return (min_x + (((y-min_y)/(max_y-min_y))*(max_x-min_x)));
}

template <class T>
void ColorMap<T>::valueToColorMap(T val, T minValue, T maxValue, T tolerance)
{
	if(val > minValue && val <= (minValue + tolerance))
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 1");
#endif		
		r = 0;
		g = 0;
		b = 255;
	}
	else if(val > (minValue + tolerance) && val <= ((maxValue-minValue)/2 -(tolerance/2)))
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 2");
#endif
		r = 0;
		g = MapValue(val, 0, 255, minValue+tolerance, ((maxValue-minValue)/2)-(tolerance/2));
		b = MapValue(val, 255, 0, minValue+tolerance, ((maxValue-minValue)/2)-(tolerance/2));
	}
	else if(val > ((maxValue-minValue)/2)-(tolerance/2) && val <= ((maxValue-minValue)/2)+(tolerance/2))
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 3");
#endif
		r = 0;
		g = 255;
		b = 0;
	}
	else if(val > ((maxValue-minValue)/2 + (tolerance/2)) && val <= (maxValue-tolerance))
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 4");
#endif		
		r = MapValue(val, 0, 255, (((maxValue-minValue)/2)+(tolerance/2)), (maxValue-tolerance));
		g = MapValue(val, 255, 0, (((maxValue-minValue)/2)+(tolerance/2)), (maxValue-tolerance));
		b = 0;
	}
	else if (val > (maxValue-tolerance) && val <= maxValue)
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 5");
#endif		
		r = 255;
		g = 0;
		b = 0;
	}
	else if(val > maxValue)
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 6");
#endif
		r = 0;
		g = 0;
		b = 0;
	}
#ifdef DEBUG	
	ROS_INFO("val: %.6f", val);
	ROS_INFO("warna : b: %d g: %d r: %d", b, g, r);
#endif	
}

template <class T>
void ColorMap<T>::valueToColorMap(T val, T minValue, T maxValue)
{
	//T tolerance = 0;
	//double half = (maxValue-minValue)/2;
	if(val > (minValue) && val < ((maxValue-minValue)/2))
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 2.1");
#endif
		r = 0;
		g = MapValue(val, 0, 255, minValue, ((maxValue-minValue)/2));
		b = MapValue(val, 255, 0, minValue, ((maxValue-minValue)/2));
	}
	else if(val > ((maxValue-minValue)/2) && val < maxValue)
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 2.2");
#endif
		r = MapValue(val, 0, 255, (((maxValue-minValue)/2)), maxValue);
		g = MapValue(val, 255, 0, ((maxValue-minValue)/2), maxValue);
		b = 0;
	}
	else if(val > maxValue)
	{
#ifdef DEBUG		
		ROS_INFO("kondisi 2.3");
#endif
		r = 0;
		g = 0;
		b = 0;
	}
#ifdef DEBUG	
	ROS_INFO("val: %.6f", val);
	ROS_INFO("warna : b: %d g: %d r: %d", b, g, r);
#endif
}

#else

inline int radiation_value(double counter)
{
	// range radiation warning to intensity:
    if(counter > 0 && counter <= background_radiation) //background
        return 0;
    else if(counter > background_radiation && counter <= radiation_warning_1)
    	return 1;
    else if(counter > radiation_warning_1 && counter <= radiation_warning_2)
        return 2;
    else if(counter > radiation_warning_2 && counter <= radiation_warning_3)
        return 3;
	else if(counter > radiation_warning_3 && counter <= radiation_warning_4)
        return 4;
	else if(counter > radiation_warning_4 && counter <= radiation_warning_5)
        return 5;
	else if(counter > radiation_warning_5 && counter <= radiation_warning_6)
        return 6;
	else if(counter > radiation_warning_6 && counter <= radiation_warning_7)
        return 7;
	else if(counter > radiation_warning_7 && counter <= radiation_warning_8)
        return 8;
	else if(counter > radiation_warning_8 && counter <= radiation_warning_9)
        return 9;
    else if(counter > radiation_warning_9)
    	return 10;
}

#endif

typedef     int     sDataPack;

#if defined (USE_CPS) && defined(USE_CPM)
    std::map<double, sDataPack, sDataPack> radiation_counter;
#else
	std::map<double, sDataPack> radiation_counter;
#endif

ros::Publisher geiger_data_cloud;
std::map<double, int> nodeStamps_;

ColorMap<double> colMapData;

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
#ifdef USE_DATA_RECORD_TO_FILE
		isDataEmpty = node.sensorData().userDataCompressed().empty();
#endif

		// unpack data mat opencv
		if(!node.sensorData().userDataCompressed().empty())
		{
			cv::Mat data;
			node.sensorData().uncompressDataConst(0 ,0, 0, &data);

#if defined (USE_CPS) && defined (USE_CPM)
			if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 3)
#else
			if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 2)
#endif
			{
				double geiger_counter = data.at<double>(0);
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembled_geiger_data(new pcl::PointCloud<pcl::PointXYZRGB>);
	int id = 0;
	for(std::map<double, int>::iterator iter=radiation_counter.begin(); iter!=radiation_counter.end(); ++iter, ++id)
	{
		double stamp_geiger_counter = iter->first;
		std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stamp_geiger_counter);
		if(previousNode!=nodeStamps.end() && previousNode->first > stamp_geiger_counter && previousNode != nodeStamps.begin())
		{
			--previousNode;
		}
		std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(stamp_geiger_counter);

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

#ifdef USE_DATA_RECORD_TO_FILE
#ifdef USE_CHRONO
			auto now   = std::chrono::high_resolution_clock::now();
			timerun = std::chrono::duration_cast<std::chrono::microseconds>(now - epoch).count();
#else
			timerun = (std::clock()-start)/(double) CLOCKS_PER_SEC;
#endif			
#endif
			rtabmap::Transform geiger_counter_pose = (poseA*v).translation();

			
#ifdef USE_COLOR_MAP
			double intensity = mapValue(iter->second, 0, max_mapped_value, 0, max_value_range);
			colMapData.valueToColorMap(intensity,0, max_mapped_value, tolerance_value);
			//colMapData.valueToColorMap(intensity,0, max_mapped_value);
#else
			int intensity = radiation_value(iter->second);
#endif
#ifdef USE_DATA_RECORD_TO_FILE
			//poseNow = geiger_counter_pose;
			//poseNow = (posePrevious*poseNow).translation();
			
			//poseNow.x() = poseNow.x()-posePrevious.x();
			//poseNow.y() = poseNow.y()-posePrevious.y();
			//poseNow.z() = poseNow.z()-posePrevious.z();

			xNow = geiger_counter_pose.x();
			yNow = geiger_counter_pose.y();
			zNow = geiger_counter_pose.z();

			xNow-=xPrev;
			yNow-=yPrev;
			zNow-=zPrev;

			xNow*=xNow;
			yNow*=yNow;
			zNow*=zNow;

			distance = sqrt(xNow +  yNow + zNow);
			distanceTemp += distance;

			if(!isDataEmpty)
			{
    			//dataFile << geiger_counter_pose.x() << "\t" << geiger_counter_pose.y() << "\t" << geiger_counter_pose.z() << "\t" << iter->second << "\t" << intensity << "\t" << std::clock()/(double) CLOCKS_PER_SEC << "\n";
				//dataFile << geiger_counter_pose.x() << "\t" << geiger_counter_pose.y() << "\t" << geiger_counter_pose.z() << "\t" << iter->second << "\t" << intensity << "\t" << distanceTemp << "\t" << std::chrono::duration_cast<std::chrono::microseconds>(now - epoch).count() << "\n";
				dataFile << geiger_counter_pose.x() << "\t" << geiger_counter_pose.y() << "\t" << geiger_counter_pose.z() << "\t" << iter->second << "\t" << intensity << "\t" << distanceTemp << "\t" << timerun  << "\t" << colMapData.r << "\t" << colMapData.g << "\t" << colMapData.b << "\n";
				//isDataEmpty = false;
			}
			//posePrevious = geiger_counter_pose;
			xPrev = geiger_counter_pose.x();
			yPrev = geiger_counter_pose.y();
			zPrev = geiger_counter_pose.z();
#endif
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			cloud->height=5;
			cloud->width=5;
			cloud->points.resize(5*5);
			for(int i=0; i<20; ++i)
			{
				// 1 cm between each points
				pcl::PointXYZRGB pt;
				pt.z = float(i+1)*0.01f;               
#ifdef USE_COLOR_MAP
				pt.r = colMapData.r;
				pt.g = colMapData.g;
				pt.b = colMapData.b;
#else
				switch(intensity)
                {
                    case 0: pt.r = 0; pt.g = 0; pt.b = 255;
                        break;
					case 1: pt.r = 0; pt.g = 55; pt.b = 200;
                        break;
                    case 2: pt.r = 0; pt.g = 105; pt.b = 150;
                        break;
					case 3: pt.r = 0; pt.g = 155; pt.b = 100;
                        break;
                    case 4: pt.r = 0; pt.g = 200; pt.b = 50;
                        break;
                    case 5: pt.r = 0; pt.g = 255; pt.b = 0;
                        break;
					case 6: pt.r = 50; pt.g = 200; pt.b = 0;
                        break;
					case 7: pt.r = 100; pt.g = 155; pt.b = 0;
                        break;
					case 8: pt.r = 150; pt.g = 100; pt.b = 0;
                        break;
					case 9: pt.r = 255; pt.g = 0; pt.b = 0;
                        break;
					case 10: pt.r = 0; pt.g = 0; pt.b = 0;
                        break;
                }
#endif
				cloud->push_back(pt);
			}
			pcl::PointXYZRGB anchor(255, 0, 0);
			cloud->push_back(anchor);

#ifdef USE_PREDICTION_CIRCLE_RANGE			
			if(intensity >= 0.22)
			{
				//float intensityFromSourcePrev = intensityByDistance(iter->second, (i+1)*0.1);
				float intensityFromSourcePrev = iter->second;
				int n = (int) intensity;
				n == 0 ? n+1 : n;
				for(int i=0; i<n; i++)
				{
					//pcl::PointXYZRGB basic_point(255, 0, 0);
					float intensityFromSource = intensityByDistance(intensityFromSourcePrev, (i+1)*0.1); //// invers square
					intensity = mapValue(intensityFromSourcePrev, 0, max_mapped_value, 0, max_value_range);
					colMapData.valueToColorMap(intensity,0, max_mapped_value, tolerance_value);
					for (float angle(0.0); angle <= 360.0; angle += 3.0)
					{
						
						pcl::PointXYZRGB basic_point;
						basic_point.x = (i+1) * 0.1 * cosf (pcl::deg2rad(angle)); // r = 10 cm.
						basic_point.y = (i+1) * 0.1 * sinf (pcl::deg2rad(angle));
						basic_point.z = 0;
						basic_point.r = colMapData.r;
						basic_point.g = colMapData.g;
						basic_point.b = colMapData.b;
						cloud->points.push_back(basic_point);
					}
					intensityFromSourcePrev = intensityFromSource;
					if(intensity<0.22)
					{	
						break;
					}	
				}
			}
			// for (float angle(0.0); angle <= 360.0; angle += 5.0)
			// {
			// 	pcl::PointXYZRGB basic_point(255, 0, 0);
			// 	basic_point.x = cosf (pcl::deg2rad(angle));
			// 	basic_point.y = sinf (pcl::deg2rad(angle));
			// 	basic_point.z = 0;
			// 	cloud->points.push_back(basic_point);
			// }
#endif
			// cloud->width = 1;
			// cloud->height = (int) cloud->points.size();

			

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

#ifdef USE_DATA_RECORD_TO_FILE
    dataFile.open (filenameData, std::fstream::out);
#endif

	std::string interface = "geiger_counter";
	double rateHz = 0.5; // Hz
	std::string frameId = "geiger_link";

	geiger_data_cloud = nh.advertise<sensor_msgs::PointCloud2>("/exa_robot/geiger_data_map_new", 1);
	ros::Subscriber mapDataSub = nh.subscribe("/rtabmap/mapData", 1, mapDataCallback);

	ros::spin();
#ifdef USE_DATA_RECORD_TO_FILE
    dataFile.close();
#endif
	return 0;
}
