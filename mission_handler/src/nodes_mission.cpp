


#include "nodes_mission.hpp"



namespace mission_handler_namespace
{
    





Node_base::Node_base(std::string name)
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










} // end mission_handler_namespace namespace


