

#ifndef ACTIONS_BT_HPP
#define ACTIONS_BT_HPP

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/blackboard.h"


#include "ros/ros.h"

//#include <chrono>
//#include <thread>


#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Vector3.h"
//#include "tf/LinearMath/Matrix3x3.h"
//#include "tf/LinearMath/Quaternion.h"



#include "mission_handler.hpp"

#include "nodes_mission.hpp"
#include "behaviortree_cpp_v3/action_node.h"



#include "definitions.hpp"




//#include <random>



/*
#define MAX_HEIGHT_MOVETO 1.8
#define TAKEOFF_HEIGHT 0.5
#define LAND_HEIGHT 0.5
#define UAV_AT_POINT_TOLERANCE 0.15
*/




namespace mission_handler_actions_namespace
{
    // add the init function to all nodes
    class SyncActionNode_custom : public BT::SyncActionNode
    {
        public:
            //using SyncActionNode::SyncActionNode;
            SyncActionNode_custom(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }
            ~SyncActionNode_custom() override = default;

            void init(mission_handler_namespace::mission_handler_state* statePtr)
            //(mission_handler_namespace::Mission_handler::mission_handler_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }


        protected:
            mission_handler_namespace::mission_handler_state* statePtr;
    };
















    // 
    class New_missions_exist : public SyncActionNode_custom
    {
        private: 

        public:

            New_missions_exist(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus tick() override
            {
                if(this->statePtr->new_missions.size() > 0)
                {
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    return BT::NodeStatus::SUCCESS;
                }
            }
    };

    // 
    class Add_new_missions : public SyncActionNode_custom
    {
        private: 

        public:

            Add_new_missions(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus tick() override
            {
                for(auto& mission : this->statePtr->new_missions)
                {
                    for(const auto& node : mission.start_nodes)
                    {
                        mission.active_nodes.push_back(node);
                        node.get()->node_start_logic(*this->statePtr);
                    }
                    this->statePtr->active_missions.push_back(mission);
                }
                this->statePtr->new_missions.clear();

                return BT::NodeStatus::SUCCESS;
            }
    };

    // 
    class Missions_are_active : public SyncActionNode_custom
    {
        private: 

        public:


            Missions_are_active(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus tick() override
            {
                if(this->statePtr->active_missions.size() > 0)
                {
                    std::cout << "Currently " << this->statePtr->active_missions.size() << " active missions" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    return BT::NodeStatus::SUCCESS;
                }
            }
    };


    // 
    class Init_new_nodes : public SyncActionNode_custom
    {
        private: 

        public:


            Init_new_nodes(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus tick() override
            {
                return BT::NodeStatus::SUCCESS;
            }
    };


// 
class Add_new_nodes_to_active_list : public SyncActionNode_custom
    {
        private: 

        public:


            Add_new_nodes_to_active_list(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus tick() override
            {
                return BT::NodeStatus::SUCCESS;
                //
            }
    };


// 
class Execute_node_logic : public SyncActionNode_custom
    {
        private: 

        public:


            Execute_node_logic(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus tick() override
            {
                for(const auto& mission : this->statePtr->active_missions)
                {
                    for(const auto& node : mission.active_nodes)
                    {
                        if(node.get()->get_current_status() == mission_handler_namespace::Node_base::RUNNING)
                        {
                            std::cout << "Node logic: " << node.get()->get_name() << std::endl;
                            node.get()->node_logic(*statePtr);
                        }
                    }
                }
                return BT::NodeStatus::SUCCESS;
            }
    };


// 
class Add_child_to_new_logic : public SyncActionNode_custom
{
    private: 

    public:


        Add_child_to_new_logic(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ };
        }

        BT::NodeStatus tick() override
        {
            for(auto& mission : this->statePtr->active_missions)
            {
                for(const auto& node : mission.active_nodes)
                {
                    // check all children to active nodes, (how to handle duplicate children???)
                    for(auto& child : node.get()->get_all_children())
                    {
                        if(child.get()->ready_to_start_logic(*this->statePtr) == true &&
                            child.get()->get_current_status() == mission_handler_namespace::Node_base::NOT_STARTED)
                        {
                            std::cout << "Node started: " << child.get()->get_name() << std::endl;
                            child.get()->node_start_logic(*this->statePtr);
                            mission.active_nodes.push_back(child);
                        }
                    }
                }
            }

            return BT::NodeStatus::SUCCESS;
            //
        }
};



// 
class Move_completed_nodes_to_finished_list : public SyncActionNode_custom
{
    private: 

    public:


        Move_completed_nodes_to_finished_list(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ };
        }

        BT::NodeStatus tick() override
        {
            return BT::NodeStatus::SUCCESS;
        }
};


// 
class Remove_completed_nodes : public SyncActionNode_custom
{
    private: 

    public:

        Remove_completed_nodes(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ };
        }

        BT::NodeStatus tick() override
        {
            for(auto& mission : this->statePtr->active_missions)
            {
                std::list<std::shared_ptr<mission_handler_namespace::Node_base>> active_nodes = mission.active_nodes;
                for(const auto& node : active_nodes)
                {

                    if(node.get()->get_current_status() == mission_handler_namespace::Node_base::COMPLETED)
                    {
                        bool remove = true;
                        // remove node when all children are completed
                        for(const auto& child : node.get()->get_all_children())
                        {
                            if(child.get()->get_current_status() != mission_handler_namespace::Node_base::COMPLETED)
                            {
                                remove = false;
                                break;
                            }
                        }
                        if(remove == true)
                        {
                            std::cout << "Node completed: " << node.get()->get_name() << std::endl;
                            mission.active_nodes.remove(node);
                        }
                    }
                } 
            }
            return BT::NodeStatus::SUCCESS;
            //
        }
};



// 
class Set_state_from_CBs : public SyncActionNode_custom
{
    private: 
    public:

        Set_state_from_CBs(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode_custom(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ };
        }

        BT::NodeStatus tick() override
        {
            // clear relevant state data
            statePtr->completed_tasks.clear();
            statePtr->added_tasks.clear();
            statePtr->results_from_tasks.clear();


            // set the current data to the accumulated data
            for(auto t : statePtr->completed_tasks_accumulated_tmp)
            {
                statePtr->completed_tasks.push_back(t);
            }
            for(auto t : statePtr->added_tasks_accumulated_tmp)
            {
                statePtr->added_tasks.push_back(t);
            }
            for(auto t : statePtr->results_from_tasks_accumulated_tmp)
            {
                statePtr->results_from_tasks.push_back(t);
            }


            statePtr->completed_tasks_accumulated_tmp.clear();
            statePtr->added_tasks_accumulated_tmp.clear();
            statePtr->results_from_tasks_accumulated_tmp.clear();


            return BT::NodeStatus::SUCCESS;
            //
        }
};
































} // end namespace
















// Template specialization to convert.
namespace BT
{

    template <> inline geometry_msgs::Pose convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Pose output;
            output.position.x    = convertFromString<double>(parts[0]);
            output.position.y    = convertFromString<double>(parts[1]);
            output.position.z    = convertFromString<double>(parts[2]);
            output.orientation.x = convertFromString<double>(parts[3]);
            output.orientation.y = convertFromString<double>(parts[4]);
            output.orientation.z = convertFromString<double>(parts[5]);
            output.orientation.w = convertFromString<double>(parts[6]);
            return output;
        }
    }

    template <> inline geometry_msgs::Point convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Point output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            return output;
        }
    }


    template <> inline bool convertFromString(StringView str)
    {
        bool output;
        if (str == "true")
        {
            output = true;
        }
        else if (str == "false")
        {
            output = false;
        }
        else
        {
            throw RuntimeError("invalid input)");
        }

        return output;
    }
} // end namespace BT








#endif





