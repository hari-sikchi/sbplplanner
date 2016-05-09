#include <sbpl_lanenav.hpp>

using namespace std;
using namespace cv;
ofstream path;


void sbplLaneNav::botpos_sub(const nav_msgs::Odometry botpos){
  bot_pos = botpos;

  tf::Quaternion q(botpos.pose.pose.orientation.x, botpos.pose.pose.orientation.y, botpos.pose.pose.orientation.z, botpos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    bot_yaw=yaw;
}

void sbplLaneNav::proptarget_sub(const geometry_msgs::PoseStamped msgtarget){
  target_pos = msgtarget;
   
  tf::Quaternion q(msgtarget.pose.orientation.x, msgtarget.pose.orientation.y, msgtarget.pose.orientation.z, msgtarget.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw=yaw;
}

void sbplLaneNav::laserscan_sub(const sensor_msgs::LaserScan msg){
  scandata = msg;
}

void sbplLaneNav::transform(){
    //Converts proposed target data from odom to base_link frame
    target_base_link.pose.position.x = target_pos.pose.position.x - bot_pos.pose.pose.position.x;
    target_base_link.pose.position.y = target_pos.pose.position.y - bot_pos.pose.pose.position.y;
    base_link_yaw = target_yaw - bot_yaw;
}

void sbplLaneNav::printdata(){
  Mat img(4480,4480,CV_8UC3,Scalar(0,0,0));
  path.open("my_env.cfg");
  string s[10];
  s[0]="discretization(cells): 4480 4480";
  s[1]="obsthresh: 1";
  s[2]="cost_inscribed_thresh: 1";
  s[3]="cost_possibly_circumscribed_thresh: 0";
  s[4]="cellsize(meters): 0.025";
  s[5]="nominalvel(mpersecs): 1.0";
  s[6]="timetoturn45degsinplace(secs): 2.0";
  s[7]="start(meters,rads): 6.95 6.95 0";
  s[8]="end(meters,rads): ";
  s[9]="environment:";
  cout<<"bot pose:"<<bot_pos.pose.pose.position.x <<" "<<bot_pos.pose.pose.position.y<<" "<<bot_yaw<<endl;
  cout<<"transform: "<<target_base_link.pose.position.x<<" "<<target_base_link.pose.position.y<<" "<<base_link_yaw<<endl;
  cout<<"target pos:"<<target_base_link.pose.position.x+bot_pos.pose.pose.position.x<<" "<<target_base_link.pose.position.y+bot_pos.pose.pose.position.y<<" "<<base_link_yaw<<endl;

 for(int i =0;i<img.rows;i++)
  {
    for(int j=0;j<img.cols;j++)
    {
      
      {
      img.at<Vec3b>(i,j)[0]=0;
      img.at<Vec3b>(i,j)[1]=0;
      img.at<Vec3b>(i,j)[2]=0;
    }
  }
}

     for(int i =0;i<img.rows;i++)
  {
    for(int j=0;j<img.cols;j++)
    {
      if(cost_map[i][j]==1)
    {
       img.at<Vec3b>(i,j)[0]=0;
      img.at<Vec3b>(i,j)[1]=255;
      img.at<Vec3b>(i,j)[2]=0;

    }
    }
  }

  //bot pose in pixel
int rel_x=2239-( (target_base_link.pose.position.x/13.975)*4480);
int rel_y=2239-( (target_base_link.pose.position.y/13.975)*4480);
cout<<rel_y<<" "<<rel_x<<endl;

  //--------
    cv::circle(img, cv::Point(rel_y, rel_x), 250, CV_RGB(255,255,255));
    cv::circle(img, cv::Point(2240, 2240), 250, CV_RGB(255,255,255));


    Size size(560,560);
    resize(img,img,size);


    imshow("lane_navigator",img);
    waitKey(5);
  path<<s[0]<<std::endl<<s[1]<<std::endl<<s[2]<<std::endl<<s[3]<<std::endl<<s[4]<<std::endl<<s[5]<<std::endl<<s[6]<<std::endl<<s[7]<<std::endl<<s[8]<<target_base_link.pose.position.x<<" "<<target_base_link.pose.position.y<<" "<<base_link_yaw<<std::endl<<s[9]<<std::endl;
  path.close();
}

void sbplLaneNav::create_costmap(){
  path.open("my_env.cfg",ios_base::app);
      int size = (scandata.angle_max - scandata.angle_min)/scandata.angle_increment;
      for(int m=0;m<size;m++){
         if(scandata.ranges[m]<scandata.range_max && scandata.ranges[m]>scandata.range_min)
         cost_map[rpos_x-(int)(40*scandata.ranges[m]*cos(scandata.angle_min + m*(scandata.angle_increment)))][rpos_y-(int)(40*scandata.ranges[m]*sin(scandata.angle_min + m*(scandata.angle_increment)))]=1;
         //cout<<"Angle min: "<<scandata.angle_min<<std::endl<<"angle_increment: "<<scandata.angle_increment<<std::endl<<"ranges: "<<scandata.ranges[m]<<std::endl;
      }
      int i,j;
      for(i=0;i<4480;i++){
         for(j=0;j<4480;j++){
            path<<cost_map[i][j]<<" ";
         }
         path<<std::endl;
      }
      path.close();
}

sbplLaneNav::sbplLaneNav(ros::NodeHandle &node_handle){
    odom_sub = node_handle.subscribe("/odometry/filtered", buffer_size, &sbplLaneNav::botpos_sub, this);
    target_sub = node_handle.subscribe("/lane_navigator/proposed_target", buffer_size, &sbplLaneNav::proptarget_sub, this);
    scan_sub = node_handle.subscribe("/scan", buffer_size, &sbplLaneNav::laserscan_sub, this);
    //transform(const nav_msgs::Odometry bot_pos, const geometry_msgs::PoseStamped target_pos, double bot_yaw, double target_yaw);
    //printdata();
}