



#ifndef OPTIMIZATION_TESTER_HPP
#define OPTIMIZATION_TESTER_HPP



#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <random>


#include "std_msgs/Float64MultiArray.h"


class Optimization_tester
{
    private:


    ros::NodeHandle nodeHandle;
    ros::NodeHandle nodeHandle_private;

    ros::Publisher result_0_publisher;
    ros::Publisher result_1_publisher;
    ros::Publisher result_2_publisher;
    //parameters?

    int number_of_agents;
    int number_of_tasks;
    double optimization_connectivity_number;


    double time_optimization(int number_of_agents, int number_of_tasks, double optimization_connectivity_number);



    public:
    Optimization_tester();
    ~Optimization_tester();

    void init_optimization_parameters(int number_of_agents, int number_of_tasks, double optimization_connectivity_number);

    void test_range(std::vector<int> number_agent_vec, std::vector<int> number_task_vec, double optimization_connectivity_number);


    void spin_loop(double loop_rate);

};












#endif