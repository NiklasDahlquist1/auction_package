
#include "mission_handler.hpp"
#include "actions_BT.hpp"





namespace mission_handler_namespace
{


Mission_handler::Mission_handler()
{
    std::cout << "AAAAAAAAAAAAAAAAAAa" <<std::endl;
    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");

    task_added_sub = nh.subscribe("/addTasks", 1000, &Mission_handler::task_added_CB, this);
    task_finished_sub = nh.subscribe("/confirmTaskFinished", 1000, &Mission_handler::task_finished_CB, this);
    task_result_sub = nh.subscribe("/task_result_TODO", 1000, &Mission_handler::task_result_CB, this);
}
Mission_handler::~Mission_handler()
{
    
}



void Mission_handler::spin_loop(double loop_rate)
{
    static bool first = true;
    if(first == true)
    {
        initialize_mission_handler();
    }


    ros::Rate rate = ros::Rate(loop_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        execute_tree(this->tree);


        /*
        if(this->state_handler.added_tasks.size() > 0)
        {
            std::cout << "Added tasks:  ";
            for(auto t : this->state_handler.added_tasks)
            {
                std::cout << t.task_ID << " ";
            }
            std::cout << std::endl;
        }
        if(this->state_handler.completed_tasks.size() > 0)
        {
            std::cout << "Completed tasks:  ";
            for(auto t : this->state_handler.completed_tasks)
            {
                std::cout << t.task_ID << " ";
            }
            std::cout << std::endl;
        }
        */

        


        rate.sleep();
    }
}




void Mission_handler::execute_tree(const BT::Tree& tree)
{
    BT::NodeStatus status;
    status = this->tree.tickRoot(); //tick behavior tree


    //std::cout << "Tree status: " << status << std::endl;
}


void Mission_handler::initialize_mission_handler()
{
    initFactory(this->factory);


    static const char* xml_text = R"(
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <ReactiveSequence>
                        <Action ID="Set_state_from_CBs"/>
                        
                        <ReactiveFallback>
                            <Action ID="New_missions_exist"/>
                            <Action ID="Add_new_missions"/>
                        </ReactiveFallback>

                        <ReactiveFallback>
                            <Action ID="Missions_are_active"/>

                            <ReactiveSequence>
                                <Action ID="Init_new_nodes"/>
                                <Action ID="Add_new_nodes_to_active_list"/>
                                <Action ID="Execute_node_logic"/>
                                <Action ID="Add_child_to_new_logic"/>
                                <Action ID="Move_completed_nodes_to_finished_list"/>
                            </ReactiveSequence>
                        </ReactiveFallback>

                    </ReactiveSequence>
                </BehaviorTree>
            </root>
            )";

    this->tree = this->factory.createTreeFromText(xml_text);


    initNodes(this->tree);
}




void Mission_handler::initNodes(BT::Tree& tree)
{
    // Iterate through all the nodes and call init() if it is an Action_B
    for( auto& node: tree.nodes )
    {
        // Not a typo: it is "=", not "=="
        if(auto set_state_from_CBs = dynamic_cast<mission_handler_actions_namespace::Set_state_from_CBs*>( node.get()))
        {
            set_state_from_CBs->init(&this->state_handler);
        }
        if(auto new_missions_exist = dynamic_cast<mission_handler_actions_namespace::New_missions_exist*>( node.get()))
        {
            new_missions_exist->init(&this->state_handler);
        }
        if(auto add_new_missions = dynamic_cast<mission_handler_actions_namespace::Add_new_missions*>( node.get()))
        {
            add_new_missions->init(&this->state_handler);
        }
        if(auto missions_are_active = dynamic_cast<mission_handler_actions_namespace::Missions_are_active*>( node.get()))
        {
            missions_are_active->init(&this->state_handler);
        }
        if(auto init_new_nodes = dynamic_cast<mission_handler_actions_namespace::Init_new_nodes*>( node.get()))
        {
            init_new_nodes->init(&this->state_handler);
        }
        if(auto add_new_nodes_to_active_list = dynamic_cast<mission_handler_actions_namespace::Add_new_nodes_to_active_list*>( node.get()))
        {
            add_new_nodes_to_active_list->init(&this->state_handler);
        }
        if(auto execute_node_logic = dynamic_cast<mission_handler_actions_namespace::Execute_node_logic*>( node.get()))
        {
            execute_node_logic->init(&this->state_handler);
        }
        if(auto add_child_to_new_logic = dynamic_cast<mission_handler_actions_namespace::Add_child_to_new_logic*>( node.get()))
        {
            add_child_to_new_logic->init(&this->state_handler);
        }
        if(auto move_completed_nodes_to_finished_list = dynamic_cast<mission_handler_actions_namespace::Move_completed_nodes_to_finished_list*>( node.get()))
        {
            move_completed_nodes_to_finished_list->init(&this->state_handler);
        }
    }
}


void Mission_handler::initFactory(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<mission_handler_actions_namespace::Set_state_from_CBs>("Set_state_from_CBs");
    factory.registerNodeType<mission_handler_actions_namespace::New_missions_exist>("New_missions_exist");
    factory.registerNodeType<mission_handler_actions_namespace::Add_new_missions>("Add_new_missions");
    factory.registerNodeType<mission_handler_actions_namespace::Missions_are_active>("Missions_are_active");
    factory.registerNodeType<mission_handler_actions_namespace::Init_new_nodes>("Init_new_nodes");
    factory.registerNodeType<mission_handler_actions_namespace::Add_new_nodes_to_active_list>("Add_new_nodes_to_active_list");
    factory.registerNodeType<mission_handler_actions_namespace::Execute_node_logic>("Execute_node_logic");
    factory.registerNodeType<mission_handler_actions_namespace::Add_child_to_new_logic>("Add_child_to_new_logic");
    factory.registerNodeType<mission_handler_actions_namespace::Move_completed_nodes_to_finished_list>("Move_completed_nodes_to_finished_list");


    //std::cout << "init nodes" << std::endl;


    return;
}





void Mission_handler::task_added_CB(const auction_msgs::taskArray& msg)
{
    for(auto task : msg.tasks)
    {
        this->state_handler.added_tasks_accumulated_tmp.push_back(task);
    }
}

void Mission_handler::task_finished_CB(const auction_msgs::task_allocated& msg)
{
    this->state_handler.completed_tasks_accumulated_tmp.push_back(msg.task);
}

void Mission_handler::task_result_CB(const auction_msgs::task_result& msg)
{
    this->state_handler.results_from_tasks_accumulated_tmp.push_back(msg);
}





















///////////////////////// mission class

Mission::Mission()
{

}
Mission::~Mission()
{

}

void Mission::add_start_node(std::shared_ptr<Node_base> start_node)
{
    this->start_nodes.push_back(start_node);
}


void Mission::print_nodes()
{
    int node_number = 0;
    for(auto start_n : this->start_nodes)
    {
        std::cout << "Mission start node number " << node_number++ << ": \n";
        std::stringstream stream;
        start_n.get()->print_mission_recursive(stream);
        std::cout << stream.str();
    }
}



} // end namespace 




