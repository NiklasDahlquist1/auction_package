
#include "mission_handler.hpp"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_handler");

    ros::NodeHandle nh;

    mission_handler_namespace::Mission_handler mission_handler;// = Mission_handler();
    


    ros::Rate r = ros::Rate(1);
    r.sleep();
    ros::spinOnce(); // delay to allow callbacks to update state, etc.

    mission_handler.spin_loop(100.0);
    

    return 0;
}





