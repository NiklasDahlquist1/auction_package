
#include "mission_handler.hpp"
#include "nodes_mission.hpp"

void create_mission(mission_handler_namespace::Mission& mission, std::string root_name);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_handler");

    ros::NodeHandle nh;

    mission_handler_namespace::Mission_handler mission_handler;// = Mission_handler();
    

    mission_handler_namespace::Mission mission;
    create_mission(mission, "root_1");
    //create_mission(mission, "root_2");
    mission.print_nodes();
    //std::stringstream stream;
    //root_node.get()->print_mission_recursive(stream);
    //std::cout << stream.str() << std::endl;
    mission_handler.addNewMission(mission);


    ros::Rate r = ros::Rate(1);
    r.sleep();
    ros::spinOnce(); // delay to allow callbacks to update state, etc.

    mission_handler.spin_loop(10.0);
    

    return 0;
}







void create_mission(mission_handler_namespace::Mission& mission, std::string root_name)
{
    auction_msgs::task task;
    task.task_name = "moveTo2D";


    std::shared_ptr<mission_handler_namespace::Node_tasks> root_node = std::make_shared<mission_handler_namespace::Node_tasks>(root_name);
    //mission_handler_namespace::Node_tasks root("root");

    std::shared_ptr<mission_handler_namespace::Node_tasks> t1 = std::make_shared<mission_handler_namespace::Node_tasks>("t1");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t2 = std::make_shared<mission_handler_namespace::Node_tasks>("t2");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t3 = std::make_shared<mission_handler_namespace::Node_tasks>("t3");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t4 = std::make_shared<mission_handler_namespace::Node_tasks>("t4");
    




    task.task_data = "-54.7;-66.3;2";
    task.task_ID = 9871;
    t1.get()->add_required_task(task);

    task.task_data = "-53.7;-66.3;2";
    task.task_ID++;
    t1.get()->add_required_task(task);

    task.task_data = "-36.5;0.4;2";
    task.task_ID++;
    t2.get()->add_required_task(task);

    task.task_data = "-26;-18.6;2";
    task.task_ID++;
    t3.get()->add_required_task(task);

    task.task_data = "-27;-18.6;2";
    task.task_ID++;
    t4.get()->add_required_task(task);




    root_node.get()->add_child(t1);
    t1.get()->add_child(t2);
    t1.get()->add_child(t3);
    t3.get()->add_child(t4);

    mission.add_start_node(root_node);



    return;
/*
    std::shared_ptr<mission_handler_namespace::Node_tasks> root_node = std::make_shared<mission_handler_namespace::Node_tasks>(root_name);
    //mission_handler_namespace::Node_tasks root("root");

    std::shared_ptr<mission_handler_namespace::Node_tasks> t1 = std::make_shared<mission_handler_namespace::Node_tasks>("t1");
    //t1.get()->add_required_task();
    std::shared_ptr<mission_handler_namespace::Node_tasks> t2 = std::make_shared<mission_handler_namespace::Node_tasks>("t2");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t3 = std::make_shared<mission_handler_namespace::Node_tasks>("t3");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t4 = std::make_shared<mission_handler_namespace::Node_tasks>("t4");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t5 = std::make_shared<mission_handler_namespace::Node_tasks>("t5");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t6 = std::make_shared<mission_handler_namespace::Node_tasks>("t6");


    root_node.get()->add_child(t1);
    t1.get()->add_child(t2);
    t1.get()->add_child(t3);
    t2.get()->add_child(t5);
    t3.get()->add_child(t4);
    t4.get()->add_child(t5);
    t5.get()->add_child(t6);

    

    mission.add_start_node(root_node);
*/



    /*
    std::cout << "parents: ";
    for(auto n : t5.get()->parents)
    {
        std::cout << n->node_name << ", ";
    }
    std::cout << std::endl;
    */
   /*
   std::cout << "leaf nodes: ";
   auto leafs = root_node.get()->find_leaf_nodes();
   for(auto leaf : leafs)
   {
    std::cout << leaf.get()->get_name() << ", ";
   }
   std::cout << std::endl;
*/


    
    
}




