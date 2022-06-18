
#ifndef MISSION_HANDLER_HPP
#define MISSION_HANDLER_HPP

#include "ros/ros.h"


#include "nodes_mission.hpp"
//#include "definitions.hpp"


#include "auction_msgs/auction.h"
#include "auction_msgs/bid.h"
#include "auction_msgs/price_bid.h"
#include "auction_msgs/task.h"
#include "auction_msgs/taskArray.h"
#include "auction_msgs/task_allocated.h"
#include "auction_msgs/task_finished.h"
#include "auction_msgs/task_result.h"


//#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"



namespace mission_handler_namespace
{


















class Mission_handler
{
    public:
        Mission_handler();
        ~Mission_handler();
        
        void initialize_mission_handler();
        void execute_tree(const BT::Tree& tree);
        void spin_loop(double loop_rate);

        //void add_mission(Node_base* start_node); //TODO





        void addNewMission(Mission mission);


    private:


        mission_handler_state state_handler;




        BT::BehaviorTreeFactory factory;
        BT::Tree tree;
        std::string tree_XML;







        // subscribers
        //ros::Subscriber newAuction_sub;
        ros::Subscriber task_added_sub;
        //ros::Subscriber taskAllocated_sub;
        ros::Subscriber task_finished_sub;
        ros::Subscriber task_result_sub;
        //ros::Subscriber new_mission_sub; // maybe create ros interface for adding (or removing, etc) mission at runtime


        // callbacks

        //void newAuction(const auction_msgs::auction& msg);
        void task_added_CB(const auction_msgs::taskArray& msg);
        //void taskAllocated(const auction_msgs::task_allocated& msg);
        void task_finished_CB(const auction_msgs::task_allocated& msg);
        void task_result_CB(const auction_msgs::task_result& msg);
        //void new_mission_CB();

        




        void initNodes(BT::Tree& tree);
        void initFactory(BT::BehaviorTreeFactory& factory);
};














} // end mission_handler_namespace namespace


#endif