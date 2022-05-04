#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <LMS1xx/LMS1xx.h>

scanData scan;

ros::Publisher* global_laserScan_pub;
std::list<sensor_msgs::LaserScan> listScans;
bool saveActivated = false;
//bool intensidad2000 = false;

void publishList() {

  if (listScans.size()>15) {
    ROS_INFO("LIST size: %i", listScans.size());
    for (sensor_msgs::LaserScan scan : listScans) {
      global_laserScan_pub->publish(scan);
    }
    listScans.clear();
  }
  //ROS_INFO("LIST INIT. size: %i", listScans.size());
}

void colectScans(const sensor_msgs::LaserScan data) {
  bool intensidad2000 = false;
  for (int i = 0; i < (data.ranges.size()-1)/2; i++) {
    if (data.intensities[i] > 2000 && data.ranges[i] < 0.5 && data.ranges[i] > 0.3) {
      if (!saveActivated) { saveActivated = true; }
      intensidad2000 = true;
      listScans.push_back(data);
      //ROS_INFO("BUCLE. valor de i: %i", i);
      //ROS_INFO("num secuecia: %i", data.header.seq);
      break;
    }
  }

  if (!intensidad2000 && saveActivated) {
    //ROS_INFO("intensidad2000: %x", intensidad2000);
    //ROS_INFO("saveActivated: %x", saveActivated);
    saveActivated = false;
    //ROS_INFO("LIST CLEAR. size: %i", listScans.size());
    publishList();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1000, colectScans);
  ros::Publisher laserScan_pub = n.advertise<sensor_msgs::LaserScan>("list_intensidad_2000", 100);
  global_laserScan_pub = &laserScan_pub;

  ros::spin();
  return 0;
}