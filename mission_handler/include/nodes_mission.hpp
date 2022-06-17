
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


    enum node_status 
    {
        COMPLETED,
        RUNNING,
        FAILED
    };

    virtual void node_logic() = 0;
    virtual void node_start_logic() = 0;
    virtual bool ready_to_start_logic() = 0;


    void add_child(std::shared_ptr<Node_base> child); // 
    void add_parent(Node_base* parent); // 
    

    void print_mission_recursive(std::stringstream& fileStream, int depth=0); // not important, but would look nice (how to fix?)
    std::string get_name();
    std::vector<std::shared_ptr<Node_base>> find_leaf_nodes();


    protected:
    std::vector<Node_base*> parents;
    std::vector<std::shared_ptr<Node_base>> children;
    std::string node_name;
    // sadfsafd status


    private:

        void indent(std::stringstream& fileStream, int depth);
};



class Node_sync : public Node_base
{
    public:
    using Node_base::Node_base;

    void node_logic();
    void node_start_logic();
    bool ready_to_start_logic();

    private:

};


class Node_tasks : public Node_base
{
    public:
    using Node_base::Node_base;

    void node_logic();
    void node_start_logic();
    bool ready_to_start_logic();


    void add_required_task(const auction_msgs::task& task);


    private:

    std::vector<auction_msgs::task> tasks_required_by_node;

    std::list<auction_msgs::task> tasks_not_finished;
    //std::vector<auction_msgs::task> tasks_to_be_added_but_not_required;
    //std::list<auction_msgs::task> tasks_finished;
};










} // end mission_handler_namespace namespace


#endif