#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include <string>

#include "LMS1xx/LMS1xx.h"
#include "console_bridge/console.h"

#include <csignal>
#include <cstdio>
#include <LMS1xx/LMS1xx.h>
#include "ros/ros.h"
#include <limits>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>  
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>  

#define DEG2RAD M_PI/180.0  

LMS1xx laser;
ros::Publisher* global_scanProcessed;
sensor_msgs::LaserScan scan_msg;

void publicarTF(float phi, double theta0, double x, double y, double z){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  // static double theta = 0.0;
  static double theta = phi;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "laser"; // CVR 2022-03-22 //  "laser_frame";

  transformStamped.transform.translation.x = x; //0.00248743;
  transformStamped.transform.translation.y = y; //-1.05702;
  transformStamped.transform.translation.z = z; //0.60373;
  tf2::Quaternion q;
  //0.0180352  0.0741931   0.053058
  q.setRPY(theta0 + theta, 0 , 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  theta=phi;
  //printf("%f\n",theta);
  br.sendTransform(transformStamped);
}

void processScanData (std_msgs::String scanStringData) {
    
    bool inf_range = false;

    ros::Time start = ros::Time::now();

    scan_msg.header.stamp = start;
    ++scan_msg.header.seq;

    scanData scan_data;
    scanCfg cfg;
    float theta;

    laser.parseScanData((char*) scanStringData.data.c_str() , &scan_data, &cfg);
    laser.debugScanData(&scan_data, &theta);

    ROS_INFO("theta: %f;  scan frec %d; points %d; msg %.1f", theta, cfg.scaningFrequency, scan_data.dist_len1, (scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment);

    int angle_range = cfg.stopAngle - cfg.startAngle;
    int num_values = angle_range / cfg.angleResolution;
    if (angle_range % cfg.angleResolution == 0)
    {
      // Include endpoint
      ++num_values;
    }

    scan_msg.header.frame_id = "laser";
    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;
    scan_msg.scan_time = 1.0 / (float) cfg.scaningFrequency;  // mal medido !!
    //scan_msg.scan_time = 0.02;
    scan_msg.angle_increment = static_cast<double>(cfg.angleResolution / 10000.0 * DEG2RAD);
    scan_msg.angle_min = static_cast<double>(cfg.startAngle / 10000.0 * DEG2RAD - M_PI / 2);
    scan_msg.angle_max = static_cast<double>(cfg.stopAngle / 10000.0 * DEG2RAD - M_PI / 2);

    scan_msg.ranges.resize(scan_data.dist_len1); // 541); //num_values
    scan_msg.intensities.resize(scan_data.dist_len1); // 541); //num_values

    scan_msg.time_increment =  /* 0.0; */
      ((float) cfg.angleResolution / 10000.0)
      / 360.0
      / ( (float) cfg.scaningFrequency / 100.0);  /* */


    for (int i = 0; i < scan_data.dist_len1; i++)
        {
          float range_data = scan_data.dist1[i] * 0.001;
    
          if (inf_range && range_data < scan_msg.range_min)
          {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
          }
          else
          {
            scan_msg.ranges[i] = range_data;
          }
        }

        for (int i = 0; i < scan_data.rssi_len1; i++)
        {
          scan_msg.intensities[i] = scan_data.rssi1[i];
        }
 
        //publicarTF(theta, theta0, x, y ,z);
        publicarTF(theta, 270.0, 0.00248743, -1.05702, 0.60373);
        
        ROS_DEBUG("Publishing scan data.");
        global_scanProcessed->publish(scan_msg);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "scanProcessor");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan_string", 1000, processScanData);
    ros::Publisher scanProcessed = n.advertise<sensor_msgs::LaserScan>("scan_processed", 1);
    global_scanProcessed = &scanProcessed;

    ros::spin();
    return 0;

}
