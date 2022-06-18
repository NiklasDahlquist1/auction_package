
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
#include "auction_msgs/task_result.h"

//#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

//#include "definitions.hpp"




namespace mission_handler_namespace
{

class Mission;

struct mission_handler_state
{
    std::vector<Mission> new_missions;
    std::list<Mission> active_missions;
    //std::vector<Mission> completed_missions; // ?

    std::vector<auction_msgs::task> completed_tasks;
    std::vector<auction_msgs::task> added_tasks;
    std::vector<auction_msgs::task_result> results_from_tasks;

    std::vector<auction_msgs::task> completed_tasks_accumulated_tmp;
    std::vector<auction_msgs::task> added_tasks_accumulated_tmp;
    std::vector<auction_msgs::task_result> results_from_tasks_accumulated_tmp;


    ros::Publisher add_task_pub;
};



class Node_base 
{
    public:
    Node_base(std::string name);
    ~Node_base();


    enum node_status 
    {
        COMPLETED,
        NOT_STARTED,
        RUNNING,
        //FAILED,
    };

    virtual void node_logic(const mission_handler_state& state) = 0;
    virtual void node_start_logic(const mission_handler_state& state) = 0;
    virtual bool ready_to_start_logic(const mission_handler_state& state) = 0;

    virtual node_status get_current_status() = 0;


    void add_child(std::shared_ptr<Node_base> child); // 
    void add_parent(Node_base* parent); // 
    

    void print_mission_recursive(std::stringstream& fileStream, int depth=0); // not important, but would look nice (how to fix?)
    std::string get_name();
    std::vector<std::shared_ptr<Node_base>> find_leaf_nodes();
    std::vector<std::shared_ptr<Node_base>> get_all_children();


    protected:
    std::vector<Node_base*> parents;
    std::vector<std::shared_ptr<Node_base>> children;
    std::string node_name;
    node_status current_status = NOT_STARTED;
    // sadfsafd status


    private:

        void indent(std::stringstream& fileStream, int depth);
};



class Node_sync : public Node_base
{
    public:
    using Node_base::Node_base;

    void node_logic(const mission_handler_state& state);
    void node_start_logic(const mission_handler_state& state);
    bool ready_to_start_logic(const mission_handler_state& state);

    node_status get_current_status();

    private:

};


class Node_tasks : public Node_base
{
    public:
    using Node_base::Node_base;

    void node_logic(const mission_handler_state& state);
    void node_start_logic(const mission_handler_state& state);
    bool ready_to_start_logic(const mission_handler_state& state);

    node_status get_current_status();


    void add_required_task(auction_msgs::task& task);


    private:

    std::vector<auction_msgs::task> tasks_required_by_node;

    std::list<auction_msgs::task> tasks_not_finished;
    //std::vector<auction_msgs::task> tasks_to_be_added_but_not_required;
    //std::list<auction_msgs::task> tasks_finished;
};








class Mission
{
    public:
    Mission();
    ~Mission();

    enum mission_status 
    {
        COMPLETED,
        IN_PROGRESS,
        FAILED
    };




    std::vector<std::shared_ptr<Node_base>> start_nodes;
    std::string mission_name;

    void add_start_node(std::shared_ptr<Node_base> start_node); 
    void print_nodes();

    mission_status check_mission_status();
    
    
    
    std::list<std::shared_ptr<Node_base>> active_nodes;


    private:
    std::vector<std::shared_ptr<Node_base>> mission_goal_nodes;
    // mission_status variable

};






} // end mission_handler_namespace namespace


#endif