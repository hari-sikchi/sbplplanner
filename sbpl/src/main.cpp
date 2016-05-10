#include <sbpl_lanenav.hpp>

int main(int argc, char* argv[]) {

	std::string node_name = "sbpl_lanenav"; 
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    sbplLaneNav *sbpl_subscriber = new sbplLaneNav(node_handle);
    ros::Rate loop_rate(10);
    while (ros::ok()){
    	
    	ros::spinOnce();
    	sbpl_subscriber->transform();
    	sbpl_subscriber->printdata();
    	sbpl_subscriber->create_costmap();
             system("cd /home/harshit/sbpl/build/");
        system("./test_sbpl /home/harshit/catkin_ws/my_env.cfg /home/harshit/sbpl/ma/home/harshit/sbpl/matlab/mprim/pr2.mprim");

    	loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
