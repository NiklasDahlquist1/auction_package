
#include "mission_handler.hpp"
#include "nodes_mission.hpp"

void create_mission(mission_handler_namespace::Mission& mission, const std::string& root_name);
void mission_scenario_easy_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_handler");

    ros::NodeHandle nh;

    mission_handler_namespace::Mission_handler mission_handler;// = Mission_handler();
    

    mission_handler_namespace::Mission mission;
    
    
    //create_mission(mission, "root_1");
    mission_scenario_easy_ifac(mission, "root_1");
    
    
    mission.print_nodes();
    //std::stringstream stream;
    //root_node.get()->print_mission_recursive(stream);
    //std::cout << stream.str() << std::endl;
    mission_handler.addNewMission(mission);

    ros::Rate r = ros::Rate(1);
    r.sleep();
    r.sleep();
    ros::spinOnce(); // delay to allow callbacks to update state, etc.

    mission_handler.spin_loop(100.0);
    

    return 0;
}







void create_mission(mission_handler_namespace::Mission& mission, const std::string& root_name)
{
    mission_handler_namespace::Node_pick_place::pick_area pick_area;
    std::vector<geometry_msgs::Point> place_points;
    pick_area.position.x = 17;
    pick_area.position.y = -41;
    pick_area.height = 4;
    pick_area.width = 4;
    geometry_msgs::Point point;
    point.x = -15;
    point.y = -31.5;
    place_points.push_back(point);
    point.x = -38;
    point.y = 6.9;
    place_points.push_back(point);
    point.x = -30.8;
    point.y = 32.2;
    place_points.push_back(point);
    point.x = -18.9;
    point.y = 31.9;
    place_points.push_back(point);
    point.x = 5;
    point.y = 33;
    place_points.push_back(point);




    mission_handler_namespace::Node_pick_place::pick_area pick_area2;
    std::vector<geometry_msgs::Point> place_points2;
    pick_area2.position.x = 0.4;
    pick_area2.position.y = 0.865;
    pick_area2.height = 0.2;
    pick_area2.width = 0.2;

    point.x = 0.36;
    point.y = -0.82;
    place_points2.push_back(point);
    point.x = -0.288;
    point.y = 0.9;
    place_points2.push_back(point);



    std::shared_ptr<mission_handler_namespace::Node_pick_place> pp_1 = std::make_shared<mission_handler_namespace::Node_pick_place>("pick_place_1");
    pp_1.get()->set_pick_area(pick_area);
    pp_1.get()->set_place_positions(place_points);
    pp_1.get()->set_parameters(0.03, 10.1, 0, 0);
    std::shared_ptr<mission_handler_namespace::Node_pick_place> pp_2 = std::make_shared<mission_handler_namespace::Node_pick_place>("pick_place_2");
    pp_2.get()->set_pick_area(pick_area2);
    pp_2.get()->set_place_positions(place_points2);
    pp_2.get()->set_parameters(1, 0.15, 0, 0);







    
    mission.add_start_node(pp_1);
//    mission.add_start_node(pp_2);


    return;


    auction_msgs::task task;
    task.task_name = "moveTo2D";


    std::shared_ptr<mission_handler_namespace::Node_tasks> root_node = std::make_shared<mission_handler_namespace::Node_tasks>(root_name);
    //mission_handler_namespace::Node_tasks root("root");

    std::shared_ptr<mission_handler_namespace::Node_tasks> t1 = std::make_shared<mission_handler_namespace::Node_tasks>("t1");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t2 = std::make_shared<mission_handler_namespace::Node_tasks>("t2");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t3 = std::make_shared<mission_handler_namespace::Node_tasks>("t3");
    std::shared_ptr<mission_handler_namespace::Node_tasks> t4 = std::make_shared<mission_handler_namespace::Node_tasks>("t4");
    std::shared_ptr<mission_handler_namespace::Node_tasks> empty_node = std::make_shared<mission_handler_namespace::Node_tasks>("empty node");





    task.task_data = "-13.9;-20.1;2";
    task.task_ID = 9871;
    t1.get()->add_required_task(task);

    task.task_data = "-6.5;-26.3;2";
    task.task_ID++;
    t1.get()->add_required_task(task);

    task.task_data = "-6.6;-12.8;2";
    task.task_ID++;
    t1.get()->add_required_task(task);

    task.task_data = "3.4;-20.4;2";
    task.task_ID++;
    t1.get()->add_required_task(task);

    task.task_data = "-5.7;-19.7;2";
    task.task_ID++;
    t1.get()->add_required_task(task);



    task.task_data = "-39.7;39.5;2";
    task.task_ID++;
    t2.get()->add_required_task(task);

    task.task_data = "-26.9;45.7;2";
    task.task_ID++;
    t2.get()->add_required_task(task);

    task.task_data = "-26;33.5;2";
    task.task_ID++;
    t2.get()->add_required_task(task);

    task.task_data = "-17.9;40;2";
    task.task_ID++;
    t2.get()->add_required_task(task);

    task.task_data = "-31.6;38;2";
    task.task_ID++;
    t2.get()->add_required_task(task);






    task.task_data = "-54.4;-64.9;2";
    task.task_ID++;
    t3.get()->add_required_task(task);

    task.task_data = "-56.4;-56.2;2";
    task.task_ID++;
    t3.get()->add_required_task(task);

    task.task_data = "-51.7;-61.1;2";
    task.task_ID++;
    t3.get()->add_required_task(task);

    task.task_data = "-42.2;-59.3;2";
    task.task_ID++;
    t3.get()->add_required_task(task);

    task.task_data = "-32.2;-61.0;2";
    task.task_ID++;
    t3.get()->add_required_task(task);



    task.task_data = "-9;1.7;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-19.9;0.1;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-6.7;12.6;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-7.6;24.6;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-25;27;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-15;20.3;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "0.4;20;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-30.1;21.1;2";
    task.task_ID++;
    t4.get()->add_required_task(task);

    task.task_data = "-25.3;2.6;2";
    task.task_ID++;
    t4.get()->add_required_task(task);


    root_node.get()->add_child(t1);
    root_node.get()->add_child(t2);
    t1.get()->add_child(empty_node);
    t2.get()->add_child(empty_node);
    empty_node.get()->add_child(t3);
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








void mission_scenario_easy_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name)
{
    mission_handler_namespace::Node_pick_place::pick_area pick_area_1;
    mission_handler_namespace::Node_pick_place::pick_area pick_area_2;
    mission_handler_namespace::Node_pick_place::pick_area pick_area_3;


    pick_area_1.position.x = 10;
    pick_area_1.position.y = -17;
    pick_area_1.height = 4;
    pick_area_1.width = 4;

    pick_area_2.position.x = 10;
    pick_area_2.position.y = -2;
    pick_area_2.height = 4;
    pick_area_2.width = 4;

    pick_area_3.position.x = 10;
    pick_area_3.position.y = 13;
    pick_area_3.height = 4;
    pick_area_3.width = 4;


    std::vector<geometry_msgs::Point> place_points;
    geometry_msgs::Point point;
    point.x = -5;
    point.y = -15;
    place_points.push_back(point);
    point.x = -5;
    point.y = 0;
    place_points.push_back(point);
    point.x = -5;
    point.y = 15;
    place_points.push_back(point);
/*    point.x = -15;
    point.y = -5;
    place_points.push_back(point);
    point.x = -15;
    point.y = 0;
    place_points.push_back(point);
    point.x = -15;
    point.y = 5;
    place_points.push_back(point);
*/




    std::shared_ptr<mission_handler_namespace::Node_pick_place> pp_1 = std::make_shared<mission_handler_namespace::Node_pick_place>("pick_place_1");
    std::shared_ptr<mission_handler_namespace::Node_pick_place> pp_2 = std::make_shared<mission_handler_namespace::Node_pick_place>("pick_place_2");
    std::shared_ptr<mission_handler_namespace::Node_pick_place> pp_3 = std::make_shared<mission_handler_namespace::Node_pick_place>("pick_place_3");

    pp_1.get()->set_pick_area(pick_area_1);
    pp_1.get()->set_place_positions(place_points);
    pp_1.get()->set_parameters(1, 0.0, 0, 10);

    pp_2.get()->set_pick_area(pick_area_2);
    pp_2.get()->set_place_positions(place_points);
    pp_2.get()->set_parameters(1, 0.0, 0, 10);

    pp_3.get()->set_pick_area(pick_area_3);
    pp_3.get()->set_place_positions(place_points);
    pp_3.get()->set_parameters(1, 0.0, 0, 10);








    mission.add_start_node(pp_1);
    mission.add_start_node(pp_2);
    mission.add_start_node(pp_3);


}


