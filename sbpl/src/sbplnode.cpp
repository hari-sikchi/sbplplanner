#include <vector>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include "ros/ros.h"
 #include <tf/transform_datatypes.h>
 #include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <iosfwd>
#include <ios>
#include <iomanip>


#define rpos_x 559
#define rpos_y 559

using namespace std;


int cost_map[1120][1120];
int i=0;
 ofstream path;

void chatterCallback2(const geometry_msgs::PoseStamped msgtarget){
   
//odom to base_link
 
  path.open("my_env.cfg");
  tf::TransformListener listener;
  geometry_msgs::PoseStamped target_base_link;
      try{
        listener.waitForTransform("/base_link", "/odom", ros::Time(0), ros::Duration(10.0) );
        listener.transformPose("/base_link", msgtarget, target_base_link);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
  
  tf::Quaternion q(target_base_link.pose.orientation.x, target_base_link.pose.orientation.y, target_base_link.pose.orientation.z, target_base_link.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw,target_yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw=yaw;

  string s[10];
  s[0]="discretization(cells): 1120 1120";
  s[1]="obsthresh: 1";
  s[2]="cost_inscribed_thresh: 1";
  s[3]="cost_possibly_circumscribed_thresh: 0";
  s[4]="cellsize(meters): 0.025";
  s[5]="nominalvel(mpersecs): 1.0";
  s[6]="timetoturn45degsinplace(secs): 2.0";
  s[7]="start(meters,rads): 13.975 13.975 0";
  s[8]="end(meters,rads): ";
  s[9]="environment:";
  path<<s[0]<<endl<<s[1]<<endl<<s[2]<<endl<<s[3]<<endl<<s[4]<<endl<<s[5]<<endl<<s[6]<<endl<<s[7]<<endl<<s[8]<<target_base_link.pose.position.x<<" "<<target_base_link.pose.position.y<<" "<<target_yaw<<endl<<s[9]<<endl;
  path.close();
}
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      path.open("my_env.cfg",ios_base::app);
      if(msg->ranges[i]<14.0)
        cost_map[rpos_x-(int)(40*msg->ranges[i]*sin(msg->angle_min + i*(msg->angle_increment)))][rpos_y+(int)(40*msg->ranges[i]*cos(msg->angle_min + i*(msg->angle_increment)))]=1;
      i++;
    
    int j;
    for(i=0;i<1120;i++){
      for(j=0;j<1120;j++)
        path<<cost_map[i][j]<<" ";
      	path<<endl;
    }
    path.close();
}

int main(int argc,char **argv){
  ros::init(argc, argv, "scan_subscriber");
    ros::NodeHandle n1,n2;
    ros::Subscriber sub1 = n2.subscribe("/lane_navigator/proposed_target", 50, chatterCallback2);
    ros::Subscriber sub = n1.subscribe("/scan", 50, chatterCallback);
    ros::spin();
    return 0;
}