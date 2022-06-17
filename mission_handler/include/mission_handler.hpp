
#ifndef MISSION_HANDLER_HPP
#define MISSION_HANDLER_HPP

#include "ros/ros.h"


#include "nodes_mission.hpp"


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
    
    
    
    


  

    private:
    std::vector<std::shared_ptr<Node_base>> mission_goal_nodes;
    // mission_status variable

};










class Mission_handler
{
    public:
        Mission_handler();
        ~Mission_handler();
        
        void initialize_mission_handler();
        void execute_tree(const BT::Tree& tree);
        void spin_loop(double loop_rate);

  //      void add_mission(Node_base* start_node); //TODO


        struct mission_handler_state
        {
            std::vector<int> new_missions;
            std::list<int> active_missions;
            std::vector<int> completed_missions;

            std::vector<auction_msgs::task> completed_tasks;
            std::vector<auction_msgs::task> added_tasks;
            std::vector<auction_msgs::task_result> results_from_tasks;

            std::vector<auction_msgs::task> completed_tasks_accumulated_tmp;
            std::vector<auction_msgs::task> added_tasks_accumulated_tmp;
            std::vector<auction_msgs::task_result> results_from_tasks_accumulated_tmp;
        };



    private:


        mission_handler_state state_handler;


        //std::list<double> active_missions;
        //std::list<double> finished_missions; ?



        BT::BehaviorTreeFactory factory;
        BT::Tree tree;
        std::string tree_XML;







        // subscribers
        //ros::Subscriber newAuction_sub;
        ros::Subscriber task_added_sub;
        //ros::Subscriber taskAllocated_sub;
        ros::Subscriber task_finished_sub;
        ros::Subscriber task_result_sub;
        //ros::Subscriber new_mission_sub;

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