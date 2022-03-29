#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void auxFunction(std::string& scanStringData) {
    
    char* varString = "Hello world from world";

    std::string auxString(varString);
    scanStringData = auxString;
    ROS_INFO("ROS INFO scanStringData: [%s]", scanStringData.c_str());
    ROS_INFO("ROS INFO auxString: [%s]", auxString.c_str());
} 

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::string stringData;

    auxFunction(stringData);

    ROS_INFO("ROS INFO stringData: [%s]", stringData.c_str());

    msg.data = stringData;

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}