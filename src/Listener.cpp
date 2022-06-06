#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include <LMS1xx/LMS1xx.h>
#include "lms1xx/ScanIterator.h"
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include "tf2_msgs/TFMessage.h"

ros::Publisher* global_laserScan_pub;
ros::Publisher* global_pointCloud_pub;
ros::Publisher* global_tf_pub;
std::list<lms1xx::ScanIterator> listScans;
std::map<int, tf2_msgs::TFMessage> mapTFs;
bool saveActivated = false;
int seqTF = 149;

laser_geometry::LaserProjection projector_;
tf::TransformListener* listener_;

void publishList() {

  if (listScans.size()>15) {
    for (int i=0; i<listScans.size(); i++) {
    //for (lms1xx::ScanIterator scaniterator : listScans) {
      auto it = listScans.begin();
      lms1xx::ScanIterator scaniterator = *it;
      it++;
      auto search = mapTFs.find(scaniterator.scan.header.seq);
      tf2_msgs::TFMessage mensaje = search->second;

      sensor_msgs::PointCloud cloud;
      projector_.transformLaserScanToPointCloud("laser", scaniterator.scan, cloud, *listener_);
      //ROS_INFO("cloud: x=%f y=%f, z=%f", cloud.points[scaniterator.iterator].x, cloud.points[scaniterator.iterator].y, cloud.points[scaniterator.iterator].z);
      //ROS_INFO("tf: x=%f y=%f, z=%f", mensaje.transforms[scaniterator.iterator].transform.translation.x, mensaje.transforms[scaniterator.iterator].transform.translation.y, mensaje.transforms[scaniterator.iterator].transform.translation.z);

      global_pointCloud_pub->publish(cloud);
      global_tf_pub->publish(mensaje);
      global_laserScan_pub->publish(scaniterator);
    }
    //ROS_INFO("tf size: %i", mapTFs.size());
    //ROS_INFO("-------------------------------------");
    listScans.clear();
  }
}

void colectScans(const sensor_msgs::LaserScan data) {
  //ROS_INFO("Data seq %u", data.header.seq);
  bool intensidad2000 = false;
  for (int i = 0; i < (data.ranges.size()-1)/2; i++) {
    if (data.intensities[i] > 2000 && data.ranges[i] < 0.5 && data.ranges[i] > 0.3) {
      if (!saveActivated) { saveActivated = true; }
      intensidad2000 = true;
      lms1xx::ScanIterator it;
      it.iterator=i;
      it.scan=data;
      listScans.push_back(it);
      break;
    }
  }

  if (!intensidad2000 && saveActivated) {
    saveActivated = false;
    publishList();
  }
}

void printTF(const tf2_msgs::TFMessage tf) {
  //mapTFs.insert(seqTF, tf);
  mapTFs.emplace(std::make_pair(seqTF, tf));
  seqTF++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1000, colectScans);
  ros::Subscriber subTF = n.subscribe("tf", 1000, printTF);
  ros::Publisher laserScan_pub = n.advertise<lms1xx::ScanIterator>("list_intensidad_2000", 100);
  ros::Publisher pointCloud_pub = n.advertise<sensor_msgs::PointCloud>("pointCloud", 100);
  ros::Publisher tf_pub = n.advertise<tf2_msgs::TFMessage>("tf_acotado", 100);
  global_laserScan_pub = &laserScan_pub;
  global_pointCloud_pub = &pointCloud_pub;
  global_tf_pub = &tf_pub;

  tf::TransformListener listener;
  listener_=&listener;

  ros::spin();
  return 0;
}