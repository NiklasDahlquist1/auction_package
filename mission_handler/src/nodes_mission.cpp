


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






//////////////// node_sync


void Node_sync::node_logic()
{
    // allways completed
}

void Node_sync::node_start_logic()
{
    // do nothing
}

bool Node_sync::ready_to_start_logic()
{
    // if all parents are OK, this is ready to start.
    return false;
}




//////////////// Node_tasks


void Node_tasks::node_logic()
{
    // check state (finished tasks), and remove them from internal queue in that case
}

void Node_tasks::node_start_logic()
{
    // publish tasks required to be completed at this stage
}

bool Node_tasks::ready_to_start_logic()
{
    // if all parents are OK, this is ready to start.
    return false;
}
void Node_tasks::add_required_task(const auction_msgs::task& task)
{
    this->tasks_required_by_node.push_back(task);
}







} // end mission_handler_namespace namespace


