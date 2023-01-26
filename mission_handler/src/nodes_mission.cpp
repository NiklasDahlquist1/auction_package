


#include "nodes_mission.hpp"



namespace mission_handler_namespace
{
    





Node_base::Node_base(const std::string& name)
{
    this->node_name = name;
}


Node_base::~Node_base()
{
    // handle memory deletion, recursively delete entire graph

    // delete children

    // delete this node
    //std::cout << "DELETING " << this->node_name << std::endl;
}


void Node_base::add_child(std::shared_ptr<Node_base> child)
{
    child->add_parent(this);
    this->children.push_back(std::move(child));
} 
void Node_base::add_parent(Node_base* parent)
{
    this->parents.push_back(parent);
} 

void Node_base::indent(std::stringstream& fileStream, int depth)
    {
        for(int i = 0; i < depth; ++i)
        {
            fileStream << "    "; //set how the indentation should be made (with 4 spaces in this case)
        }
    }


void Node_base::print_mission_recursive(std::stringstream& fileStream, int depth)
{
    indent(fileStream, depth);

    fileStream << this->node_name << "\n";

    for(const auto& child : this->children)
    {
        child->print_mission_recursive(fileStream, depth + 1);
    }
}

std::string Node_base::get_name()
{
    return this->node_name;
}



std::vector<std::shared_ptr<Node_base>> Node_base::find_leaf_nodes()
{
    std::vector<std::shared_ptr<Node_base>> leaf_nodes;
    // check if children are leaf nodes, if so return them else call function on it
    for(auto child : this->children)
    {
        if(child.get()->children.size() == 0)
        {
            leaf_nodes.push_back(child);
        }
        else
        {
            for(auto c : child.get()->find_leaf_nodes())
            {
                bool leaf_is_unique = true;
                // check to make sure that c is not already in list
                for(auto existing_leaf : leaf_nodes)
                {
                    if(c == existing_leaf)
                    {
                        leaf_is_unique = false;
                        break;
                    }
                }
                if(leaf_is_unique == true)
                {
                    leaf_nodes.push_back(c);
                }

            }
        }
    }
    
    return leaf_nodes;
}



std::vector<std::shared_ptr<Node_base>> Node_base::get_all_children()
{
    return this->children;
}







//////////////// node_sync


void Node_sync::node_logic(const mission_handler_state& state)
{
    // allways completed
    this->current_status = COMPLETED;
}

void Node_sync::node_start_logic(const mission_handler_state& state)
{
    // do nothing
    this->current_status = COMPLETED;
}

bool Node_sync::ready_to_start_logic(const mission_handler_state& state)
{
    // if all parents are OK, this is ready to start.
    // if number of parents = 0, also ready to start

    bool ready = true;
    for(const auto& parent : this->parents)
    {
        if(parent->get_current_status() != Node_base::node_status::COMPLETED)
        {
            ready = false;
        }
    }
    return ready;
}
Node_base::node_status Node_sync::get_current_status()
{
    return this->current_status;
}



//////////////// Node_tasks




void Node_tasks::node_logic(const mission_handler_state& state)
{
    // check state (finished tasks), and remove them from internal queue in that case

    for(const auction_msgs::task& completed_task : state.completed_tasks)
    {
        // compare and check if the completed task exists in this nodes not completed tasks, if remove it from this node
        for(const auction_msgs::task& internal_task : this->tasks_not_finished)
        {
            if(completed_task == internal_task)
            {
                this->tasks_not_finished.remove(internal_task);
                break;
            }
        }
    }


    // set current node state
    if(tasks_not_finished.size() == 0)
    {
        this->current_status = COMPLETED;
    }
    else
    {
        this->current_status = RUNNING;
    }

}


void Node_tasks::node_start_logic(const mission_handler_state& state)
{
    // publish tasks required to be completed at this stage

    auction_msgs::taskArray tasks;
    for(const auction_msgs::task& task : this->tasks_required_by_node)
    {
        tasks.tasks.push_back(task);
        this->tasks_not_finished.push_back(task);
    }
    if(tasks.tasks.size() > 0)
    {
        state.add_task_pub.publish(tasks);
    }


    // set current node state
    this->current_status = RUNNING;
}


bool Node_tasks::ready_to_start_logic(const mission_handler_state& state)
{
    // if all parents are OK, this is ready to start.
    // if number of parents = 0, also ready to start

    bool ready = true;
    for(const auto& parent : this->parents)
    {
        if(parent->get_current_status() != Node_base::node_status::COMPLETED)
        {
            ready = false;
        }
    }
    return ready;
}


Node_base::node_status Node_tasks::get_current_status()
{
    return this->current_status;
}


void Node_tasks::add_required_task(auction_msgs::task& task)
{
    task.creator_name = "Mission, node: " + this->node_name;
    this->tasks_required_by_node.push_back(task);
}















//////////////// Node_pick_place




void Node_pick_place::node_logic(const mission_handler_state& state)
{
    // increase time, check if we should add a task, create a task (with random pick/place position)
    double dt = ros::Time::now().toSec() - this->time_last_check;
    time_accumulator += dt;
    this->time_last_check = ros::Time::now().toSec();



    while(this->time_accumulator > this->spawn_interval)
    //if(ros::Time::now().toSec() - this->time_last_check > spawn_interval) // very approximate, time error will accumulate over time. but who cares
    {
        std::uniform_real_distribution<> dist_01(0.0, 1.0); // distribution
        if(dist_01(this->rng) < this->spawn_rate)
        {   
            add_task(state);
        }

        
        this->time_accumulator -= this->spawn_interval;
    }

}


void Node_pick_place::node_start_logic(const mission_handler_state& state)
{
    this->time_last_check = ros::Time::now().toSec();
    // we do nothing here?


    // add start tasks
    for(int i = 0; i < this->number_of_start_tasks; ++i)
    {
        add_task(state);
    }

    // set current node state
    this->current_status = RUNNING;
}


bool Node_pick_place::ready_to_start_logic(const mission_handler_state& state)
{
    // if all parents are OK, this is ready to start.
    // if number of parents = 0, also ready to start

    bool ready = true;
    for(const auto& parent : this->parents)
    {
        if(parent->get_current_status() != Node_base::node_status::COMPLETED)
        {
            ready = false;
        }
    }
    return ready;
}


Node_base::node_status Node_pick_place::get_current_status()
{
    return this->current_status;
}



void Node_pick_place::set_pick_area(const Node_pick_place::pick_area& area)
{
    this->pick_area_task = area;
}
void Node_pick_place::set_place_positions(const std::vector<geometry_msgs::Point>& place_points)
{
    this->place_points = place_points;
}
void Node_pick_place::set_parameters(double spawn_interval, double spawn_rate, double task_reward, int number_of_start_tasks)
{
    this->spawn_interval = spawn_interval;
    this->spawn_rate = spawn_rate;
    this->task_reward = task_reward;

    this->number_of_start_tasks = number_of_start_tasks;
}


void Node_pick_place::add_task(const mission_handler_state& state)
{
    geometry_msgs::Point pick_point;
    geometry_msgs::Point place_point;

    std::uniform_real_distribution<> dist_01(0.0, 1.0); // distribution

    pick_point.x = this->pick_area_task.position.x + this->pick_area_task.width * dist_01(this->rng);
    pick_point.y = this->pick_area_task.position.y + this->pick_area_task.height * dist_01(this->rng);


    std::uniform_int_distribution<> dist_place(0, this->place_points.size() - 1);
    int place_point_index = dist_place(this->rng);
    place_point.x = this->place_points[place_point_index].x;
    place_point.y = this->place_points[place_point_index].y;



    std::uniform_int_distribution<> dist_id(0, INT32_MAX);


    auction_msgs::task task;
    task.task_data = std::to_string(pick_point.x) + ";" + std::to_string(pick_point.y) + ";" +
                        std::to_string(place_point.x) + ";" + std::to_string(place_point.y);
    task.created_time = ros::Time::now();
    task.creator_name = this->node_name;
    task.task_ID = dist_id(this->rng);
    task.reward = this->task_reward;
    task.task_name = "pickPlace";

    auction_msgs::taskArray tasks;
    tasks.tasks.push_back(task);
    state.add_task_pub.publish(tasks);
}









///////////////////////// Node_multiuple_pick_place














void Node_multiple_pick_place::node_logic(const mission_handler_state& state)
{


    // check state (finished tasks), and remove them from internal queue in that case

    for(const auction_msgs::task& completed_task : state.completed_tasks)
    {
        // compare and check if the completed task exists in this nodes not completed tasks, if remove it from this node
        for(const auction_msgs::task& internal_task : this->added_tasks)
        {
            if(completed_task == internal_task)
            {
                this->added_tasks.remove(internal_task);
                break;
            }
        }
    }


    // increase time, check if we should add a task, create a task (with random pick/place position)


    // loop through vector in random order
    std::random_shuffle (this->pick_areas_task.begin(), this->pick_areas_task.end() );  // in place no extra array

    // do for each pick up place
    for(auto& pp : this->pick_areas_task)
    {
        double dt = ros::Time::now().toSec() - pp.time_last_check;
        pp.time_accumulator += dt;
        pp.time_last_check = ros::Time::now().toSec();

        



        while(pp.time_accumulator > pp.parameters.spawn_interval)
        //if(ros::Time::now().toSec() - this->time_last_check > spawn_interval) // very approximate, time error will accumulate over time. but who cares
        {
            //std::cout << "TRYING TO SPAWN\n";
            std::uniform_real_distribution<> dist_01(0.0, 1.0); // distribution
            if(dist_01(this->rng) <= pp.parameters.spawn_rate)
            {   
                if(this->added_tasks.size() < this->max_number_of_active_tasks) // should be tested once somewhere else, but who cares. This is easier since all stations get their time accumulator reduces properly
                {
                    add_task(state, pp);
                }
            }
            pp.time_accumulator -= pp.parameters.spawn_interval;
        }
    }
}


void Node_multiple_pick_place::node_start_logic(const mission_handler_state& state)
{    
    for(auto& pp : this->pick_areas_task)
    {
        for(int i = 0; i < pp.parameters.number_of_start_tasks; ++i)
        {
            if(this->added_tasks.size() < this->max_number_of_active_tasks)
            {
                add_task(state, pp);
            }
        }
        
        pp.time_last_check = ros::Time::now().toSec();
        pp.time_accumulator = 0;
    }

    
    // we do nothing here?


    // add start tasks


    // set current node state
    this->current_status = RUNNING;
}


bool Node_multiple_pick_place::ready_to_start_logic(const mission_handler_state& state)
{
    // if all parents are OK, this is ready to start.
    // if number of parents = 0, also ready to start

    bool ready = true;
    for(const auto& parent : this->parents)
    {
        if(parent->get_current_status() != Node_base::node_status::COMPLETED)
        {
            ready = false;
        }
    }
    return ready;
}


Node_base::node_status Node_multiple_pick_place::get_current_status()
{
    return this->current_status;
}



void Node_multiple_pick_place::add_pick_area(const Node_multiple_pick_place::pick_area& area)
{
    this->pick_areas_task.push_back(area);
    // add function to remove area? maybe not relevant? (not now anyway)
}

void Node_multiple_pick_place::set_max_active_tasks(int max)
{
    this->max_number_of_active_tasks = max;
}



void Node_multiple_pick_place::add_task(const mission_handler_state& state, const pick_area& pick_area)
{
    geometry_msgs::Point pick_point;
    geometry_msgs::Point place_point;

    std::uniform_real_distribution<> dist_01(0.0, 1.0); // distribution

    pick_point.x = pick_area.position.x + pick_area.width * dist_01(this->rng);
    pick_point.y = pick_area.position.y + pick_area.height * dist_01(this->rng);


    std::uniform_int_distribution<> dist_place(0, pick_area.place_points.size() - 1);
    int place_point_index = dist_place(this->rng);
    place_point.x = pick_area.place_points[place_point_index].x;
    place_point.y = pick_area.place_points[place_point_index].y;



    std::uniform_int_distribution<> dist_id(0, INT32_MAX);


    auction_msgs::task task;
    task.task_data = std::to_string(pick_point.x) + ";" + std::to_string(pick_point.y) + ";" +
                        std::to_string(place_point.x) + ";" + std::to_string(place_point.y);
    task.created_time = ros::Time::now();
    task.creator_name = this->node_name;;
    //task.task_type = "";
    task.task_ID = dist_id(this->rng);
    task.reward = pick_area.parameters.task_reward;
    task.task_name = "pickPlace";
    
    task.constrained_group = pick_area.parameters.use_constrain_groups;
    task.constrained_group_name = pick_area.parameters.station_name;


    // should we consider time here?
    task.useIncreasingReward = pick_area.parameters.use_increasing_reward;


    auction_msgs::taskArray tasks;
    tasks.tasks.push_back(task);
    state.add_task_pub.publish(tasks);

    this->added_tasks.push_back(task);
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
    for(const auto& start_n : this->start_nodes)
    {
        std::cout << "Mission start node number " << node_number++ << ": \n";
        std::stringstream stream;
        start_n.get()->print_mission_recursive(stream);
        std::cout << stream.str();
    }
}










} // end mission_handler_namespace namespace


