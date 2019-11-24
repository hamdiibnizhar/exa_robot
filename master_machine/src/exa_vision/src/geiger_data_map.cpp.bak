#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>

typedef     long int     sDataPack;

struct geiger_pack
{
  sDataPack cps = 0;
  sDataPack cpm = 0;
};

geiger_pack geiger_sensor;

std::map<sDataPack, sDataPack, double> radiation_counter;
void mapDataCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg)
{
   rtabmap::Transform mapToOdom;
   std::map<int, rtabmap::Transform> poses;
   std::multimap<int, rtabmap::Link> links;
   std::map<int, rtabmap::Signature> signatures;

   rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);
   for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
   {
      cv::Mat data;
      iter->second.sensorData().uncompressDataConst(0, 0, 0, 0, &data);
      if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 3)
      {
         sDataPack cps = data.at<sDataPack>(0);
         sDataPack cpm = data.at<sDataPack>(1);
         double stamp = data.at<double>(2);
         radiation_counter.insert(std::make_pair(stamp, cps, cpm));
      }
      else if(!data.empty())
      {
         ROS_ERROR("Wrong user data format for geiger signal transfer.");
      }
   }

}



int main(int argc, char** argv)
{
   //... node initialization stuff

   ros::init(argc, argv, "geiger_data_convert");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
   
   ros::Rate rate(0.5);  
   // nh.init(argc, argv, "geiger_counter");

   ros::Subscriber mapDataSub = nh.subscribe("exa_robot/detector_data_map", 1, mapDataCallback);

   ros::spin();
}