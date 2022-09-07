



#ifndef TASK_VISUALIZER_HPP
#define TASK_VISUALIZER_HPP



#include "ros/ros.h"

#include "auction_msgs/auction.h"
#include "auction_msgs/bid.h"
#include "auction_msgs/price_bid.h"
#include "auction_msgs/task.h"
#include "auction_msgs/taskArray.h"
#include "auction_msgs/task_allocated.h"
#include "auction_msgs/task_finished.h"

#include "std_msgs/String.h"

#include <visualization_msgs/Marker.h>



#include <string_view>



namespace task_visualization
{
    std::vector<std::string> splitString(const std::string &strToSplit, char delimeter);


    class Task_visualizer
    {

        private:

        // pub 
        ros::Publisher currentAuctionTasks_pub;
        ros::Publisher addedtasks_pub;
        ros::Publisher tasksCurrentlyAllocated_pub; // tasks that are actively allocated?
        ros::Publisher finishedTasks_pub;
        ros::Publisher tasksNotFinished_pub;

        // subscribers
        ros::Subscriber newAuction_sub;
        ros::Subscriber taskAdded_sub;
        ros::Subscriber taskAllocated_sub;
        ros::Subscriber taskFinished_sub;
        // callbacks

        void newAuction(const auction_msgs::auction& msg);
        void taskAdded(const auction_msgs::taskArray& msg);
        void taskAllocated(const auction_msgs::task_allocated& msg);
        void taskFinished(const auction_msgs::task_allocated& msg);



        auction_msgs::auction latestAuction;
        std::vector<auction_msgs::task> allAddedTasks;
        std::map<int, auction_msgs::task_allocated> currentlyAllocatedTasks;
        std::map<int, auction_msgs::task> tasksCurrentlyNotFinished;
        std::vector<auction_msgs::task> allFinishedTasks;



        
        public:

        Task_visualizer();

        void pubCurrentlyNotFinishedTasks();
        void pubAlltasksAdded();
        void pubFinishedTasks();
        void pubMarkers();

        void pubCurrentlyAllocatedTasks();

    };

    void Task_visualizer::pubMarkers()
    {
        pubFinishedTasks();
        pubAlltasksAdded();
        pubCurrentlyNotFinishedTasks();
    }

    void Task_visualizer::pubFinishedTasks()
    {
        // finished tasks
        visualization_msgs::Marker allAdded;

        
        std::vector<geometry_msgs::Point> task_pos;

        for(const auction_msgs::task& t : allFinishedTasks)
        {
            if(t.task_name == "moveTo" || t.task_name == "moveTo2D") 
            {
                geometry_msgs::Point point;
                auto parts = splitString(t.task_data, ';');
                point.x = std::stod(parts[0]);
                point.y = std::stod(parts[1]);
                point.z = std::stod(parts[2]);
                task_pos.push_back(point);
            }
            else if(t.task_name == "pickPlace")
            {
                geometry_msgs::Point point;
                auto parts = splitString(t.task_data, ';');
                point.x = std::stod(parts[2]);
                point.y = std::stod(parts[3]);
                point.z = 0;
                task_pos.push_back(point);

            }


        }
        //allAdded.lifetime
        allAdded.header.frame_id = "world";
        allAdded.header.stamp = ros::Time::now();
        allAdded.ns = "auction_visualizer";
        allAdded.id = 1;
        allAdded.type = visualization_msgs::Marker::CUBE_LIST;
        allAdded.action = visualization_msgs::Marker::ADD;
        allAdded.pose.position.x = 0;
        allAdded.pose.position.y = 0;
        allAdded.pose.position.z = 0;
        allAdded.pose.orientation.x = 0.0;
        allAdded.pose.orientation.y = 0.0;
        allAdded.pose.orientation.z = 0.0;
        allAdded.pose.orientation.w = 1.0;
        allAdded.scale.x = 0.75*0.3;
        allAdded.scale.y = 0.75*0.3;
        allAdded.scale.z = 0.4*0.3;
        allAdded.color.a = 0.95;
        allAdded.color.r = 0.0;
        allAdded.color.g = 1.0;
        allAdded.color.b = 0.0;
        allAdded.points = task_pos;
        finishedTasks_pub.publish(allAdded);
    }







  void Task_visualizer::pubCurrentlyNotFinishedTasks()
    {

        // all not finished
        visualization_msgs::Marker allAdded;
        
        visualization_msgs::Marker marker_blue;


        std::vector<geometry_msgs::Point> task_blue;
        std::vector<geometry_msgs::Point> task_pos;

        for(auto pair : tasksCurrentlyNotFinished)
        {
            if(pair.second.task_name == "moveTo" || pair.second.task_name == "moveTo2D")
            {
                geometry_msgs::Point point;
                auto parts = splitString(pair.second.task_data, ';');
                point.x = std::stod(parts[0]);
                point.y = std::stod(parts[1]);
                point.z = std::stod(parts[2]);
                task_pos.push_back(point);
            }
            else if(pair.second.task_name == "pickPlace")
            {
                auto parts = splitString(pair.second.task_data, ';');

                geometry_msgs::Point pick_point;
                pick_point.x = std::stod(parts[0]);
                pick_point.y = std::stod(parts[1]);
                pick_point.z = 0;
                task_pos.push_back(pick_point);

                geometry_msgs::Point place_point;
                place_point.x = std::stod(parts[2]);
                place_point.y = std::stod(parts[3]);
                place_point.z = 0;
                task_blue.push_back(place_point);

            }

        }
        //allAdded.lifetime
        allAdded.header.frame_id = "world";
        allAdded.header.stamp = ros::Time::now();
        allAdded.ns = "auction_visualizer";
        allAdded.id = 1;
        allAdded.type = visualization_msgs::Marker::CUBE_LIST;
        allAdded.action = visualization_msgs::Marker::ADD;
        allAdded.pose.position.x = 0;
        allAdded.pose.position.y = 0;
        allAdded.pose.position.z = 0;
        allAdded.pose.orientation.x = 0.0;
        allAdded.pose.orientation.y = 0.0;
        allAdded.pose.orientation.z = 0.0;
        allAdded.pose.orientation.w = 1.0;
        allAdded.scale.x = 0.75*0.3;
        allAdded.scale.y = 0.75*0.3;
        allAdded.scale.z = 0.4*0.3;
        allAdded.color.a = 0.95;
        allAdded.color.r = 1.0;
        allAdded.color.g = 0.0;
        allAdded.color.b = 0.0;
        allAdded.points = task_pos;
        tasksNotFinished_pub.publish(allAdded);



        marker_blue.header.frame_id = "world";
        marker_blue.header.stamp = ros::Time::now();
        marker_blue.ns = "auction_visualizer";
        marker_blue.id = 2;
        marker_blue.type = visualization_msgs::Marker::CUBE_LIST;
        marker_blue.action = visualization_msgs::Marker::ADD;
        marker_blue.pose.position.x = 0;
        marker_blue.pose.position.y = 0;
        marker_blue.pose.position.z = 0;
        marker_blue.pose.orientation.x = 0.0;
        marker_blue.pose.orientation.y = 0.0;
        marker_blue.pose.orientation.z = 0.0;
        marker_blue.pose.orientation.w = 1.0;
        marker_blue.scale.x = 0.75*0.3;
        marker_blue.scale.y = 0.75*0.3;
        marker_blue.scale.z = 0.4*0.3;
        marker_blue.color.a = 0.95;
        marker_blue.color.r = 0.0;
        marker_blue.color.g = 0.0;
        marker_blue.color.b = 1.0;
        marker_blue.points = task_blue;
        tasksNotFinished_pub.publish(marker_blue);

    }



    void Task_visualizer::pubCurrentlyAllocatedTasks()
    {

        // all not finished
        visualization_msgs::Marker allAdded;

        
        std::vector<geometry_msgs::Point> task_pos;

        for(auto pair : currentlyAllocatedTasks)
        {
            if(pair.second.task.task_name == "moveTo" || pair.second.task.task_name == "moveTo2D")
            {
                geometry_msgs::Point point;
                auto parts = splitString(pair.second.task.task_data, ';');
                point.x = std::stod(parts[0]);
                point.y = std::stod(parts[1]);
                point.z = std::stod(parts[2]);
                task_pos.push_back(point);
            }

        }
        //allAdded.lifetime
        allAdded.header.frame_id = "world";
        allAdded.header.stamp = ros::Time::now();
        allAdded.ns = "auction_visualizer";
        allAdded.id = 1;
        allAdded.type = visualization_msgs::Marker::CUBE_LIST;
        allAdded.action = visualization_msgs::Marker::ADD;
        allAdded.pose.position.x = 0;
        allAdded.pose.position.y = 0;
        allAdded.pose.position.z = 0.01;
        allAdded.pose.orientation.x = 0.0;
        allAdded.pose.orientation.y = 0.0;
        allAdded.pose.orientation.z = 0.0;
        allAdded.pose.orientation.w = 1.0;
        allAdded.scale.x = 1.5*0.3;
        allAdded.scale.y = 1.5*0.3;
        allAdded.scale.z = 1*0.3;
        allAdded.color.a = 0.95;
        allAdded.color.r = 0.0;
        allAdded.color.g = 0.0;
        allAdded.color.b = 1.0;
        allAdded.points = task_pos;
        tasksCurrentlyAllocated_pub.publish(allAdded);

        // todo: make this much better. now, clear all tasks every x secs....
        static double lastClearTime = ros::Time::now().toSec();
        double timeNow = ros::Time::now().toSec();
        if((timeNow - lastClearTime) > 1)
        {
            lastClearTime = timeNow;
            std::vector<int> ids;
            for(auto remove : currentlyAllocatedTasks)
            {
                ids.push_back(remove.first);
            }
            for(int i : ids)
            {
                currentlyAllocatedTasks.erase(i);
            }
        }
    }



    void Task_visualizer::pubAlltasksAdded()
    {
        // all added tasks
        visualization_msgs::Marker allAdded;

        
        std::vector<geometry_msgs::Point> task_pos;

        for(const auction_msgs::task& t : allAddedTasks)
        {
            if(t.task_name == "moveTo" || t.task_name == "moveTo2D")
            {
                geometry_msgs::Point point;
                auto parts = splitString(t.task_data, ';');
                point.x = std::stod(parts[0]);
                point.y = std::stod(parts[1]);
                point.z = std::stod(parts[2]);
                task_pos.push_back(point);
            }

        }
        //allAdded.lifetime
        allAdded.header.frame_id = "world";
        allAdded.header.stamp = ros::Time::now();
        allAdded.ns = "auction_visualizer";
        allAdded.id = 1;
        allAdded.type = visualization_msgs::Marker::CUBE_LIST;
        allAdded.action = visualization_msgs::Marker::ADD;
        allAdded.pose.position.x = 0;
        allAdded.pose.position.y = 0;
        allAdded.pose.position.z = 0;
        allAdded.pose.orientation.x = 0.0;
        allAdded.pose.orientation.y = 0.0;
        allAdded.pose.orientation.z = 0.0;
        allAdded.pose.orientation.w = 1.0;
        allAdded.scale.x = 0.2;
        allAdded.scale.y = 0.2;
        allAdded.scale.z = 0.2;
        allAdded.color.a = 0.9;
        allAdded.color.r = 0.0;
        allAdded.color.g = 0.0;
        allAdded.color.b = 1.0;
        allAdded.points = task_pos;
        addedtasks_pub.publish(allAdded);

    }





    Task_visualizer::Task_visualizer()
    {
        ros::NodeHandle nh;
        currentAuctionTasks_pub = nh.advertise<visualization_msgs::Marker>("/visualizer/currentAuction", 1000);
        addedtasks_pub = nh.advertise<visualization_msgs::Marker>("/visualizer/addedTasks", 1000);
        tasksCurrentlyAllocated_pub = nh.advertise<visualization_msgs::Marker>("/visualizer/activeTasks", 1000); // tasks that are actively allocated?
        finishedTasks_pub = nh.advertise<visualization_msgs::Marker>("/visualizer/finishedTasks", 1000);
        tasksNotFinished_pub = nh.advertise<visualization_msgs::Marker>("/visualizer/tasksNotFinished", 1000);

        // subscribers
        newAuction_sub = nh.subscribe("auctionAvailable", 100, &Task_visualizer::newAuction, this);
        taskAdded_sub = nh.subscribe("addTasks", 100, &Task_visualizer::taskAdded, this);
        taskAllocated_sub = nh.subscribe("allocatedTasks", 100, &Task_visualizer::taskAllocated, this);
        taskFinished_sub = nh.subscribe("confirmTaskFinished", 100, &Task_visualizer::taskFinished, this);
        // callbacks
    }

    void Task_visualizer::newAuction(const auction_msgs::auction& msg)
    {
        //
        latestAuction = msg;
    }
    void Task_visualizer::taskAdded(const auction_msgs::taskArray& msg)
    {
        allAddedTasks.insert(allAddedTasks.end(), msg.tasks.begin(), msg.tasks.end());

        for(auction_msgs::task t : msg.tasks)
        {
            tasksCurrentlyNotFinished[t.task_ID] = t;
        }
    }
    void Task_visualizer::taskAllocated(const auction_msgs::task_allocated& msg)
    {
        //currentlyAllocatedTasks.erase(msg.task.task_ID);
        currentlyAllocatedTasks[msg.task.task_ID] = msg;
    }
    void Task_visualizer::taskFinished(const auction_msgs::task_allocated& msg)
    {
        allFinishedTasks.push_back(msg.task);


        


        for(auto pair : tasksCurrentlyNotFinished)
        {
            if(pair.first == msg.task.task_ID)
            {
                tasksCurrentlyNotFinished.erase(pair.first);
                break;
            }
        }

    }












std::vector<std::string> splitString(const std::string &strToSplit, char delimeter)
{
    std::vector<std::string> splitted_strings;
    splitted_strings.reserve(4);

    size_t pos = 0;
    while( pos < strToSplit.size())
    {
        size_t new_pos = strToSplit.find_first_of(delimeter, pos);
        if( new_pos == std::string::npos)
        {
           new_pos = strToSplit.size();
        }
        std::string sv = { &strToSplit.data()[pos], new_pos - pos };
        splitted_strings.push_back( sv );
        pos = new_pos + 1;
    }
    return splitted_strings;
} 

} // end namespace auction



#endif