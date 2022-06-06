





#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

#include <visualization_msgs/Marker.h>

#include "auction_msgs/taskArray.h"



ros::Publisher centrumMarkers_pub;


ros::Publisher task_pub;
int task_ID = 121312; // fix later




void reAddTaskWhenFinished(const auction_msgs::task& msg)
{
    auction_msgs::task task;
    //task.task_name = "moveTo2D";
    //task.task_data = std::to_string(msg.point.x) + ";" + std::to_string(msg.point.y) + ";" + std::to_string(msg.point.z);

    task.task_ID = task_ID;
    task_ID++;



    auction_msgs::taskArray taskArray;
    taskArray.tasks.push_back(task);
    task_pub.publish(taskArray);

    std::cout << "Recieved point" << std::endl;

}



void addAllTasks()
{
    double xMin = -40;
    double xMax = 40;
    int xNumMarkers = 10;

    double yMin = -40;
    double yMax = 40;
    int yNumMarkers = 10;

    double height = 2;

    int startID = 123213; // fix

    visualization_msgs::Marker allAdded;
    std::vector<geometry_msgs::Point> taskCentrum;


    auction_msgs::taskArray tasks;
    

    for(int i = 0; i < xNumMarkers; ++i)
    {
        double xP = (xNumMarkers - i - 1) * 1.0 / (xNumMarkers - 1);
        for(int j = 0; j < yNumMarkers; ++j)
        {
            geometry_msgs::Point point;
            point.x = xMin + xP * (xMax - xMin);
            
            double yP = (yNumMarkers - j - 1) * 1.0 / (yNumMarkers - 1);
            point.y = yMin + yP * (yMax - yMin);

            point.z = 0;
            
            taskCentrum.push_back(point);

            auction_msgs::task t;
            t.created_time = ros::Time::now();
            t.task_ID = startID;
            startID += 1;
            t.task_name = "moveTo";
            t.task_data = std::to_string(point.x) + ";" + std::to_string(point.y) + ";" + std::to_string(height);
            t.reward = (pow(xMax - xMin, 3) - sqrt(pow(point.x - 0, 2) + pow(point.y - 0, 2))) * 30;

            tasks.tasks.push_back(t);
        }
        //std::cout << xP << std::endl;
    }

    //allAdded.lifetime
    allAdded.header.frame_id = "map";
    allAdded.header.stamp = ros::Time::now();
    allAdded.ns = "visualization";
    allAdded.id = 157415;
    allAdded.type = visualization_msgs::Marker::CUBE_LIST;
    allAdded.action = visualization_msgs::Marker::ADD;
    allAdded.pose.position.x = 0;
    allAdded.pose.position.y = 0;
    allAdded.pose.position.z = 0;
    allAdded.pose.orientation.x = 0.0;
    allAdded.pose.orientation.y = 0.0;
    allAdded.pose.orientation.z = 0.0;
    allAdded.pose.orientation.w = 1.0;
    allAdded.scale.x = (xMax - xMin) / (xNumMarkers - 1);
    allAdded.scale.y = (yMax - yMin) / (yNumMarkers - 1);
    allAdded.scale.z = 0.5;
    allAdded.color.a = 0.95;
    allAdded.color.r = 0.0;
    allAdded.color.g = 0.0;
    allAdded.color.b = 1.0;
    allAdded.points = taskCentrum;

    centrumMarkers_pub.publish(allAdded);
    task_pub.publish(tasks);
}






int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_adder");

    ros::NodeHandle nh;

    task_pub = nh.advertise<auction_msgs::taskArray>("/addTasks", 1000);
    centrumMarkers_pub = nh.advertise<visualization_msgs::Marker>("/visualization/gridTasks", 1000);

    //ros::Subscriber reAddTtask_sub = nh.subscribe("/confirmee...", 100, reAddTaskWhenFinished);
    ros::Rate delay = ros::Rate(2);
    delay.sleep();
    addAllTasks();

    ros::Rate rate = ros::Rate(1);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }



    return 0;
}
