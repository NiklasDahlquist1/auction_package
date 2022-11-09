


#include "optimization_tester.hpp"
#include "optimization_ortools.h"





Optimization_tester::Optimization_tester()
{
    result_fixed_agents_0_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("optimization_tester_result_fixed_agents_0", 1000);
    result_fixed_agents_1_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("optimization_tester_result_fixed_agents_1", 1000);
    result_fixed_agents_2_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("optimization_tester_result_fixed_agents_2", 1000);

    result_fixed_tasks_0_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("optimization_tester_result_fixed_tasks_0", 1000);
    result_fixed_tasks_1_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("optimization_tester_result_fixed_tasks_1", 1000);
    result_fixed_tasks_2_publisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("optimization_tester_result_fixed_tasks_2", 1000);
}
Optimization_tester::~Optimization_tester()
{

}

void Optimization_tester::init_optimization_parameters(int number_of_agents, int number_of_tasks, double optimization_connectivity_number)
{
    this->number_of_agents = number_of_agents;
    this->number_of_tasks = number_of_tasks;
    this->optimization_connectivity_number = optimization_connectivity_number;
}

double Optimization_tester::time_optimization(int number_of_agents, int number_of_tasks, double optimization_connectivity_number, ros::Publisher& result_publisher)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<> dist(0, 1);


    // create matrix for optimization
  // task matching, assigns the maximum number of tasks at the minimum cost
  // cost[i][j], i=worker, j=cost associated with task j
  // returns array result[i][k] with k tasks associated with each worker i.  empty if no tasks             ////   if -1 that worker has no task assigned
  // rewards[j], reward associated with finishing task j
  // maxNumOfTasksAssigned: the number of tasks that can be assigned to a single agent

    std::vector<std::vector<double>> costs(number_of_agents, std::vector<double>(number_of_tasks)); 
    std::vector<double> rewards(number_of_tasks);
    int max_number_of_tasks_assigned = 1;


    double max_cost = 10;
    int num_connections = 0;
    for(auto& tasks : costs)
    {
        for(auto& task : tasks)
        {
            if(dist(rng) < optimization_connectivity_number)
            {
                task = dist(rng) * max_cost;
                num_connections += 1;
            }
            else
            {
                task = -1;
            }
        }
    }

    /*std::cout << "Cost matrix (casted to ints for visualization): " << std::endl;
    for(int i = 0; i < number_of_agents; ++i)
    {
        for(int j = 0; j < number_of_tasks; ++j)
        {
            std::cout << (int) costs[i][j] << "\t";
        }
        std::cout << std::endl;
    }*/
    //std::cout << "\nConnectrions: " << num_connections << "\n";


    // print optimization parameters (variables. constraints, etc.)




    //std::cout << "Starting solvers\n" << std::flush;
    auto start = std::chrono::high_resolution_clock::now();

    // call solvers
    operations_research::taskMatching(costs, rewards, max_number_of_tasks_assigned, "SCIP"); //CP-SAT CBC SCIP

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end-start;
    std::cout << "Solving time: " << elapsed.count() << " ms\n";

    std_msgs::Float64MultiArray res;

    res.data.push_back(elapsed.count());
    res.data.push_back(number_of_agents);
    res.data.push_back(number_of_tasks);
    res.data.push_back(num_connections);
    result_publisher.publish(res);




    return 0;

/*
    start = std::chrono::high_resolution_clock::now();
    operations_research::taskMatching(costs, rewards, max_number_of_tasks_assigned, "SCIP"); //use the matching optimization
    end = std::chrono::high_resolution_clock::now();
    elapsed = end-start;
    std::cout << "Solving time: " << elapsed.count() << " ms\n";

    res = std_msgs::Float64MultiArray();
    res.data.push_back(elapsed.count());
    res.data.push_back(number_of_agents);
    res.data.push_back(number_of_tasks);
    res.data.push_back(num_connections);
    result_1_publisher.publish(res);


    return 0;

    start = std::chrono::high_resolution_clock::now();
    operations_research::taskMatching(costs, rewards, max_number_of_tasks_assigned, "SCIP"); //use the matching optimization
    end = std::chrono::high_resolution_clock::now();
    elapsed = end-start;
    std::cout << "Solving time: " << elapsed.count() << " ms\n";

    res = std_msgs::Float64MultiArray();
    res.data.push_back(elapsed.count());
    res.data.push_back(number_of_agents);
    res.data.push_back(number_of_tasks);
    res.data.push_back(num_connections);
    result_2_publisher.publish(res);
    

    return 0;
*/



}

void Optimization_tester::spin_loop(double loop_rate)
{
    ros::Rate rate = ros::Rate(loop_rate);
    

    
    
    
    while (ros::ok())
    {
        ros::spinOnce();

        time_optimization(number_of_agents, number_of_tasks, optimization_connectivity_number, result_fixed_tasks_0_publisher);

        rate.sleep();
    }
}



void Optimization_tester::task_loop()
{
    
    // 
    int agent_step = 10;
    int task_step = 10;

    int num_agents = 50;
    int num_tasks = 50;
    int max_tasks = 500;
    int max_agents = 500;



    //for(int a = agent_step; a <= this->number_of_agents; a += agent_step)
    //{
        for(int t = task_step; t <= max_tasks; t += task_step)
        {
            time_optimization(num_agents, t, 0.3, result_fixed_agents_0_publisher);
        }
    //}
        for(int t = task_step; t <= max_tasks; t += task_step)
        {
            time_optimization(num_agents, t, 0.6, result_fixed_agents_1_publisher);
        }
        for(int t = task_step; t <= max_tasks; t += task_step)
        {
            time_optimization(num_agents, t, 0.9, result_fixed_agents_2_publisher);
        }




    for(int a = agent_step; a <= max_agents; a += agent_step)
    {
        //for(int t = task_step; t <= max_tasks; t += task_step)
        //{
            time_optimization(a, num_tasks, 0.3, result_fixed_tasks_0_publisher);
        //}
    }

    for(int a = agent_step; a <= max_agents; a += agent_step)
    {
        //for(int t = task_step; t <= max_tasks; t += task_step)
        //{
            time_optimization(a, num_tasks, 0.6, result_fixed_tasks_1_publisher);
        //}
    }

    for(int a = agent_step; a <= max_agents; a += agent_step)
    {
        //for(int t = task_step; t <= max_tasks; t += task_step)
        //{
            time_optimization(a, num_tasks, 0.9, result_fixed_tasks_2_publisher);
        //}
    }




    return;
}







int main(int argc, char** argv)
{
    ros::init(argc, argv, "optimization_tester");

    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");


    int number_of_agents;
    int number_of_tasks;
    double optimization_connectivity_number; // how many tasks are connected to an agent (\in [0, 1])




    /*if (!nh_private.getParam ("number_of_agents", number_of_agents))
        number_of_agents = 50;
    if (!nh_private.getParam ("number_of_tasks", number_of_tasks))
        number_of_tasks = 1000;
    if (!nh_private.getParam ("optimization_connectivity_number", optimization_connectivity_number))*/
        optimization_connectivity_number = 0.01;



    //std::cout << "Optimization parameters: \n" << "\tnumber of agents: " << number_of_agents << "\n\tnumber of tasks: " << number_of_tasks 
    //          << "\n\toptimization connectivity number: " << optimization_connectivity_number << "\n";




    Optimization_tester optimization_tester = Optimization_tester();
    optimization_tester.init_optimization_parameters(number_of_agents, number_of_tasks, optimization_connectivity_number);
    optimization_tester.task_loop();

    return 0;
}





