#include <map>
#include "task_visualizer.hpp"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "auction_visualizer");

    ros::NodeHandle nh;



    task_visualization::Task_visualizer visualizer = task_visualization::Task_visualizer();
    
        ros::Rate rate = ros::Rate(10);
        while (ros::ok())
        {
            ros::spinOnce();

            visualizer.pubMarkers();
            visualizer.pubCurrentlyNotFinishedTasks();
            visualizer.pubCurrentlyAllocatedTasks();
            rate.sleep();
        }






    /*std::map<int, char> tasks;

    tasks[1] = 'a';
    tasks[12] = 'b';

    std::cout << tasks[1] << " : " << tasks[12] << " : " << tasks[12] << std::endl;
    std::cout << tasks.size() << std::endl;
    tasks.erase(12);
    std::cout << tasks.size() << std::endl;

    for(auto a : tasks)
    {
        std::cout << "map: " << a.second << std::endl;
    }*/


    return 0;
}





