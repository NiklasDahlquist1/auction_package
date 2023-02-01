
#include "mission_handler.hpp"
#include "nodes_mission.hpp"

void create_mission(mission_handler_namespace::Mission& mission, const std::string& root_name);
void mission_scenario_easy_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name);
void mission_scenario_big_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name);
void mission_scenario_final_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name);
void abb_demo_mission(mission_handler_namespace::Mission& mission, const std::string& root_name);




int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_handler");

    ros::NodeHandle nh;

    mission_handler_namespace::Mission_handler mission_handler;// = Mission_handler();
    

    mission_handler_namespace::Mission mission;
    
    
    //create_mission(mission, "root_1");
    //mission_scenario_easy_ifac(mission, "easy_scenario");
    //mission_scenario_big_ifac(mission, "big_map");
    //mission_scenario_final_ifac(mission, "final_scenario");

    abb_demo_mission(mission, "abb_demo");
    
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
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_1;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_2;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_3;

    pick_area_1.position.x = 10;
    pick_area_1.position.y = -20;
    pick_area_1.height = 5;
    pick_area_1.width = 5;
    pick_area_1.parameters.number_of_start_tasks = 0;
    pick_area_1.parameters.station_name = "station_1";
    pick_area_1.parameters.spawn_interval = 1;
    pick_area_1.parameters.spawn_rate = 0.05;
    pick_area_1.parameters.task_reward = 0;

    pick_area_2.position.x = 10;
    pick_area_2.position.y = -2.5;
    pick_area_2.height = 5;
    pick_area_2.width = 5;
    pick_area_2.parameters.number_of_start_tasks = 0;
    pick_area_2.parameters.station_name = "station_2";
    pick_area_2.parameters.spawn_interval = 1;
    pick_area_2.parameters.spawn_rate = 0.15;
    pick_area_2.parameters.task_reward = 0;

    pick_area_3.position.x = 10;
    pick_area_3.position.y = 15;
    pick_area_3.height = 5;
    pick_area_3.width = 5;
    pick_area_3.parameters.number_of_start_tasks = 0;
    pick_area_3.parameters.station_name = "station_3";
    pick_area_3.parameters.spawn_interval = 1;
    pick_area_3.parameters.spawn_rate = 0.15;
    pick_area_3.parameters.task_reward = 0;

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

    pick_area_1.place_points = place_points;
    pick_area_2.place_points = place_points;
    pick_area_3.place_points = place_points;


    std::shared_ptr<mission_handler_namespace::Node_multiple_pick_place> pp_1 = std::make_shared<mission_handler_namespace::Node_multiple_pick_place>("MULTI_PICK_PLACE_1");
    pp_1.get()->add_pick_area(pick_area_1);
    pp_1.get()->add_pick_area(pick_area_2);
    pp_1.get()->add_pick_area(pick_area_3);

    pp_1.get()->set_max_active_tasks(40);







    mission.add_start_node(pp_1);
    return;
}





void mission_scenario_final_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name)
{
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_1;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_2;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_3;

    pick_area_1.position.x = -13;
    pick_area_1.position.y = -12;
    pick_area_1.height = 5;
    pick_area_1.width = 5;
    pick_area_1.parameters.number_of_start_tasks = 0;
    pick_area_1.parameters.station_name = "station_1";
    pick_area_1.parameters.spawn_interval = 1;
    pick_area_1.parameters.spawn_rate = 0.05;
    pick_area_1.parameters.task_reward = 0;

    pick_area_2.position.x = -2.5;
    pick_area_2.position.y = -12;
    pick_area_2.height = 5;
    pick_area_2.width = 5;
    pick_area_2.parameters.number_of_start_tasks = 0;
    pick_area_2.parameters.station_name = "station_2";
    pick_area_2.parameters.spawn_interval = 1;
    pick_area_2.parameters.spawn_rate = 0.15;
    pick_area_2.parameters.task_reward = 0;

    pick_area_3.position.x = 7;
    pick_area_3.position.y = -12;
    pick_area_3.height = 5;
    pick_area_3.width = 5;
    pick_area_3.parameters.number_of_start_tasks = 0;
    pick_area_3.parameters.station_name = "station_3";
    pick_area_3.parameters.spawn_interval = 1;
    pick_area_3.parameters.spawn_rate = 0.15;
    pick_area_3.parameters.task_reward = 0;

    std::vector<geometry_msgs::Point> place_points;
    geometry_msgs::Point point;
    point.x = -10;
    point.y = 4;
    place_points.push_back(point);
    point.x = -1;
    point.y = 11;
    place_points.push_back(point);
    point.x = 8.5;
    point.y = 11.5;
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

    pick_area_1.place_points = place_points;
    pick_area_2.place_points = place_points;
    pick_area_3.place_points = place_points;


    std::shared_ptr<mission_handler_namespace::Node_multiple_pick_place> pp_1 = std::make_shared<mission_handler_namespace::Node_multiple_pick_place>("MULTI_PICK_PLACE_1");
    pp_1.get()->add_pick_area(pick_area_1);
    pp_1.get()->add_pick_area(pick_area_2);
    pp_1.get()->add_pick_area(pick_area_3);

    pp_1.get()->set_max_active_tasks(40);







    mission.add_start_node(pp_1);
    return;
}







void mission_scenario_big_ifac(mission_handler_namespace::Mission& mission, const std::string& root_name)
{
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_1;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_2;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_3;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_4;

    pick_area_1.position.x = 23;
    pick_area_1.position.y = -42;
    pick_area_1.height = 4;
    pick_area_1.width = 4;
    pick_area_1.parameters.number_of_start_tasks = 5;
    pick_area_1.parameters.station_name = "station_1";
    pick_area_1.parameters.spawn_interval = 2;
    pick_area_1.parameters.spawn_rate = 0.0;
    pick_area_1.parameters.task_reward = 0;

    pick_area_2.position.x = 27;
    pick_area_2.position.y = -42;
    pick_area_2.height = 4;
    pick_area_2.width = 4;
    pick_area_2.parameters.number_of_start_tasks = 5;
    pick_area_2.parameters.station_name = "station_2";
    pick_area_2.parameters.spawn_interval = 2;
    pick_area_2.parameters.spawn_rate = 0.0;
    pick_area_2.parameters.task_reward = 0;

    pick_area_3.position.x = 36.5;
    pick_area_3.position.y = 123;
    pick_area_3.height = 4;
    pick_area_3.width = 4;
    pick_area_3.parameters.number_of_start_tasks = 5;
    pick_area_3.parameters.station_name = "station_3";
    pick_area_3.parameters.spawn_interval = 2;
    pick_area_3.parameters.spawn_rate = 0.0;
    pick_area_3.parameters.task_reward = 0;

    pick_area_4.position.x = 40.5;
    pick_area_4.position.y = 123;
    pick_area_4.height = 4;
    pick_area_4.width = 4;
    pick_area_4.parameters.number_of_start_tasks = 5;
    pick_area_4.parameters.station_name = "station_4";
    pick_area_4.parameters.spawn_interval = 2;
    pick_area_4.parameters.spawn_rate = 0.0;
    pick_area_4.parameters.task_reward = 0;




    std::vector<geometry_msgs::Point> place_points;
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
    point.x = 41;
    point.y = 58;
    place_points.push_back(point);
    point.x = 41;
    point.y = 78.5;
    place_points.push_back(point);
    point.x = 40.5;
    point.y = 99;
    place_points.push_back(point);
    point.x = -34.5;
    point.y = 139.5;
    place_points.push_back(point);


    pick_area_1.place_points = place_points;
    pick_area_2.place_points = place_points;
    pick_area_3.place_points = place_points;
    pick_area_4.place_points = place_points;


    std::shared_ptr<mission_handler_namespace::Node_multiple_pick_place> pp_1 = std::make_shared<mission_handler_namespace::Node_multiple_pick_place>("MULTI_PICK_PLACE_1");
    pp_1.get()->add_pick_area(pick_area_1);
    pp_1.get()->add_pick_area(pick_area_2);
    pp_1.get()->add_pick_area(pick_area_3);
    pp_1.get()->add_pick_area(pick_area_4);

    //pp_1.get()->add_pick_area(pick_area_3);

    pp_1.get()->set_max_active_tasks(20);







    mission.add_start_node(pp_1);
    return;
}














void abb_demo_mission(mission_handler_namespace::Mission& mission, const std::string& root_name)
{
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_1;
    mission_handler_namespace::Node_multiple_pick_place::pick_area pick_area_2;


    pick_area_1.position.x = 1.2;
    pick_area_1.position.y = 1.75;
    pick_area_1.height = 0.9;
    pick_area_1.width = 0.9;
    pick_area_1.parameters.number_of_start_tasks = 0;
    pick_area_1.parameters.station_name = "station_1";
    pick_area_1.parameters.spawn_interval = 0.5;
    pick_area_1.parameters.spawn_rate = 0.04;
    pick_area_1.parameters.task_reward = 0;

    pick_area_2.position.x = -1.9;
    pick_area_2.position.y = 0.9;
    pick_area_2.height = 0.9;
    pick_area_2.width = 0.9;
    pick_area_2.parameters.number_of_start_tasks = 0;
    pick_area_2.parameters.station_name = "station_2";
    pick_area_2.parameters.spawn_interval = 0.5;
    pick_area_2.parameters.spawn_rate = 0.04;
    pick_area_2.parameters.task_reward = 0;



    std::vector<geometry_msgs::Point> place_points;
    geometry_msgs::Point point;
    point.x = -2.1;
    point.y = 3.2;
    place_points.push_back(point);
    point.x = -2.1;
    point.y = 5.4;
    place_points.push_back(point);
    point.x = 1.9;
    point.y = 5.4;
    place_points.push_back(point);
    point.x = 1.6;
    point.y = 4.25;
    place_points.push_back(point);
/*    point.x = -15;
    point.y = 0;
    place_points.push_back(point);
    point.x = -15;
    point.y = 5;
    place_points.push_back(point);
*/



    pick_area_1.place_points = place_points;
    pick_area_2.place_points = place_points;


    std::shared_ptr<mission_handler_namespace::Node_multiple_pick_place> pp_1 = std::make_shared<mission_handler_namespace::Node_multiple_pick_place>("ABB_PICK_PLACE_MISSION");
    pp_1.get()->add_pick_area(pick_area_1);
    pp_1.get()->add_pick_area(pick_area_2);



    pp_1.get()->set_max_active_tasks(15);







    mission.add_start_node(pp_1);
    return;
}











