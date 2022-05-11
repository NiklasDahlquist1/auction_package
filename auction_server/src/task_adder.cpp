


#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"


#include "auction_msgs/taskArray.h"



ros::Publisher task_pub;
int task_ID = 121312; // fix later


void rvizPointCB(const geometry_msgs::PointStamped& msg)
{
    auction_msgs::task task;
    task.task_name = "moveTo2D";
    task.task_data = std::to_string(msg.point.x) + ";" + std::to_string(msg.point.y) + ";" + std::to_string(msg.point.z);

    task.task_ID = task_ID;
    task_ID++;



    auction_msgs::taskArray taskArray;
    taskArray.tasks.push_back(task);
    task_pub.publish(taskArray);

    std::cout << "Recieved point" << std::endl;

}






int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_adder");

    ros::NodeHandle nh;

    task_pub = nh.advertise<auction_msgs::taskArray>("/addTasks", 1000);

    ros::Subscriber rvizPoint_sub = nh.subscribe("/clicked_point", 100, rvizPointCB);



    ros::Rate rate = ros::Rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }








    return 0;
}





