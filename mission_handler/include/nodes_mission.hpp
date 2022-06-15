
#ifndef NODES_MISSION_HPP
#define NODES_MISSION_HPP

#include "ros/ros.h"


#include "auction_msgs/auction.h"
#include "auction_msgs/bid.h"
#include "auction_msgs/price_bid.h"
#include "auction_msgs/task.h"
#include "auction_msgs/taskArray.h"
#include "auction_msgs/task_allocated.h"
#include "auction_msgs/task_finished.h"


//#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"



namespace mission_handler_namespace
{


class Node_base 
{
    public:
    Node_base(std::string name);
    ~Node_base();


    virtual void node_logic();
    virtual void node_start_logic();
    virtual bool ready_to_start_logic();


    void add_children(const Node_base* children); // 

    //void print_mission_recursive(); // not important, but would look nice (how to fix?)

    protected:
    std::vector<Node_base*> parents;
    std::vector<Node_base*> children;
    std::string node_name;
    // sadfsafd status


    private:
};



class Node_sync : Node_base
{
    public:

    private:

};


class Node_tasks : Node_base
{
    public:

    private:


    std::list<auction_msgs::task> tasks_not_finished;
    //std::list<auction_msgs::task> tasks_finished;
};






} // end mission_handler_namespace namespace


#endif