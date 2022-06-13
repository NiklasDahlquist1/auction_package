

#include "auction_server.hpp"
#include "optimization_ortools.h"


namespace auction_ns
{
    Auction_server::Auction_server()
    {
        auctionAvailable_pub = nodeHandle.advertise<auction_msgs::auction>("auctionAvailable", 1000);
        auctionWinners_pub = nodeHandle.advertise<auction_msgs::task_allocated>("allocatedTasks", 1000);
        taskFinished_pub = nodeHandle.advertise<auction_msgs::task_allocated>("confirmTaskFinished", 1000);
        taskNotAllocated_pub = nodeHandle.advertise<std_msgs::String>("notAllocated", 1000);


        addTasks_sub = nodeHandle.subscribe("addTasks", 100, &Auction_server::addTasksCB, this);
        bids_sub = nodeHandle.subscribe("submitBid", 100, &Auction_server::bidsCB, this);
        taskFinished_sub = nodeHandle.subscribe("confirmTaskFinished", 100, &Auction_server::taskFinishedCB, this);
        newClient_sub = nodeHandle.subscribe("addNewClient", 100, &Auction_server::newClientCB, this);


        std::cout << "Auction server started, node: " << ros::this_node::getName() << std::endl;
    }
    Auction_server::~Auction_server()
    {
    }

    void Auction_server::initServerParameters(double auctionRoundTime)
    {
        this->auctionRoundTime = auctionRoundTime;
    }

    void Auction_server::createAndPublishNewAuction()
    {
        auction_msgs::auction newAuction;
        newAuction.header.stamp = ros::Time::now();

        // add all tasks from the queue
        for(task_allocated t : taskQueue)
        {
            newAuction.tasks.tasks.push_back(t.task);
        }
        newAuction.auction_ID = auctionIDCounter;
        auctionIDCounter += 1;
        

        // set newAuction as activeAuction
        activeAuction = newAuction;

        auctionAvailable_pub.publish(activeAuction);

        std::cout << "Auction round starting, " << activeAuction.tasks.tasks.size() << " tasks available" << std::endl;

    }

    void Auction_server::updateLogic()
    {
        if(auctionRunning)
        {
            double currentTime = ros::Time::now().toSec();
            if(activeAuction.header.stamp.toSec() + auctionRoundTime <= currentTime || bidsCurrentAuction.size() >= 3) //TODO
            {
                declareWinner();
                auctionRunning = false;
            }
        }
        else if(taskQueue.size() >= 1 && auctionRunning != true) // start new auction if there is atleast one task available and auction is not already active
        {
            createAndPublishNewAuction();

            auctionRunning = true;            
        }    
    }
    void Auction_server::spin_loop(double loop_rate)
    {
        ros::Rate rate = ros::Rate(loop_rate);
        while (ros::ok())
        {
            ros::spinOnce();

            updateLogic();
            rate.sleep();
        }
    }



    void Auction_server::bidsCB(const auction_msgs::bid& msg)
    {
        //only accept bid if it has the ID of the current auction
        
        if(msg.auction_ID == this->activeAuction.auction_ID)
        {
            this->bidsCurrentAuction.push_back(msg);
        }
        else
        {
            std::cout << "Did not accept bid from " << msg.agent_name << ", auctionID is not correct or outdated" << std::endl;
        }
    }

    // callback implementations
    void Auction_server::addTasksCB(const auction_msgs::taskArray& msg)
    {

        for(auction_msgs::task task : msg.tasks)
        {
            Auction_server::task_allocated task_a;
            task_a.task = task;
            task_a.allocated = false;
            task_a.allocatedTo = "";
            taskQueue.push_back(task_a);
        }
        // TODO: maybe check so that ID is unique?


/*
        std::cout << "tasks in queue:" << std::endl;
        for(auto t : taskQueue)
        {
            std::cout << "    task name: " << t.task.task_name << std::endl;
        }
*/
    }

    void Auction_server::taskFinishedCB(const auction_msgs::task_allocated& msg)
    {
        task_allocated taskToRemove;
        bool removeFound = false;
        for(task_allocated t : taskQueue)
        {
            //maybe we should check who sent this?



            // TODO: handle if the task is allocated to another client?
            if(t.allocated == true)
            {
                if(t.allocatedTo != t.allocatedTo)
                {
                    // publish task finished again to allow for the agent to drop the task?
                    taskFinished_pub.publish(msg);
                }   

            }

            if(t.task == msg.task)
            {
                removeFound = true;
                taskToRemove = t;
                break;
            }
        }

        if(removeFound)
        {
            taskQueue.remove(taskToRemove);
            //std::cout << "Task finished: " << taskToRemove.task.task_name << std::endl;

        }
        else
        {
            //std::cout << msg.task_name << " not in queue?" << std::endl;
        }

    }

    void Auction_server::newClientCB(const std_msgs::String& msg)
    {
        bool clientAlreadyExists = false; //TODO, implement

        if(clientAlreadyExists == false)
        {
            this->clientsRegistered.push_back(msg.data);
        }
        else
        {
            // print warning
        }

        std::cout << "new client, current clients:" << std::endl;
        for(std::string s : this->clientsRegistered)
        {
            std::cout << "    name: " << s << std::endl;
        }
    }








    void Auction_server::declareWinner()
    {

        if (bidsCurrentAuction.size() == 0)
        {
            //zero bids
            std::cout << "Zero bids, no allocations" << std::endl;
            return; //return if there are no bids, ie. no winners to be calculated
        }

        // TODO: do we handle min/max costs here?

        std::vector<std::string> agentsThatAreAllocated; // can we make this nicer?
        std::vector<std::string> agentsThatWasAllocated; // for checking if an agent lost his task and is no longer assigned to any


        auction_msgs::auction auctionForAssigning;
        auctionForAssigning.auction_ID = activeAuction.auction_ID;
        auctionForAssigning.header = activeAuction.header;
        for(task_allocated& t : taskQueue)
        {
            // TODO, filter out some tasks depending on if the current owner has responded or not, if it can be reassign or not, etc. 


            // save name of agents that already had a task allocated
            if(t.allocated == true)
            {
                agentsThatWasAllocated.push_back(t.allocatedTo);
            }
            // we set all tasks that are up for auctioning to be "not allocated"
            t.allocated = false;
            t.allocatedTo = "";


            auctionForAssigning.tasks.tasks.push_back(t.task);
        }

        // create cost matrix (to allow old optimization function TODO, maybe send the data in a better format?)
        int workers_num = bidsCurrentAuction.size();
        int tasks_num = auctionForAssigning.tasks.tasks.size();
        //std::cout << "workers: " << workers_num << " tasks: " << tasks_num << std::endl;
        std::vector<std::vector<double>> costs(workers_num, std::vector<double>(tasks_num)); //TODO check if every bid is from a unique worker






        // create rewards vector
        std::vector<double> rewards(tasks_num);
        for(int j = 0; j < tasks_num; ++j)
        {
            rewards[j] = auctionForAssigning.tasks.tasks[j].reward;
        }


        // maybe not optimal?
        // do for each recieved bid
        for(int i = 0; i < bidsCurrentAuction.size(); ++i)
        {
            for(int j = 0; j < auctionForAssigning.tasks.tasks.size(); ++j)
            {
                for(auction_msgs::price_bid price : bidsCurrentAuction[i].prices)
                {
                    bool pairFound = false;
                    if(auctionForAssigning.tasks.tasks[j].task_ID == price.task_ID)
                    {
                        pairFound = true;
                        costs[i][j] = price.price;
                        break;
                    }
                    if(pairFound == false)
                    {
                        costs[i][j] = -1;
                    }
                }
            }
        }



        /*std::cout << "Cost matrix: " << std::endl;
        for(int i = 0; i < workers_num; ++i)
        {
            for(int j = 0; j < tasks_num; ++j)
            {
                std::cout << costs[i][j] << " ";
            }
            std::cout << std::endl;
        }*/

        
        std::vector<std::vector<double>> winners(workers_num);
        winners = operations_research::taskMatching(costs, rewards, 5); //use the matching optimization




        for(int i = 0; i < winners.size(); ++i)
        {
            for(int t : winners[i])
            {
                std::cout << t << " ";
            }
            std::cout <<"; ";
        }
        std::cout << std::endl;


        std::cout << "Auction round ended, " << bidsCurrentAuction.size() << " bids received" << std::endl;








        for(int i = 0; i < winners.size(); ++i)
        {
            bool agentHaveAllocatedTask = false;
            std::vector<auction_msgs::task_allocated> tasks_allocated;
            std::vector<double> associatedCosts;
            int index = 0;

            for(double winners_value : winners[i])
            {


                if(winners_value > -0.5)
                {
                    auction_msgs::task_allocated task_allocated;

                    task_allocated.task = auctionForAssigning.tasks.tasks[winners[i][index]];
                    task_allocated.agent_ID = bidsCurrentAuction[i].agent_ID;
                    task_allocated.agent_name = bidsCurrentAuction[i].agent_name;
                    task_allocated.time_allocated = ros::Time::now();

                    tasks_allocated.push_back(task_allocated);
                    associatedCosts.push_back(costs[i][winners[i][index]]);  //bidsCurrentAuction[i].prices[index].price);




                    std::list<auction_ns::Auction_server::task_allocated>::iterator it;
                    auction_ns::Auction_server::task_allocated t_win;
                    t_win.task = auctionForAssigning.tasks.tasks[winners[i][index]];
                    it = std::find(taskQueue.begin(), taskQueue.end(), t_win);

                    index += 1;



                    (*it).allocated = true;
                    it->allocatedTo = bidsCurrentAuction[i].agent_name;
                    agentHaveAllocatedTask = true;
                }
            }

            //
            if(agentHaveAllocatedTask == true)
            {
                agentsThatAreAllocated.push_back(bidsCurrentAuction[i].agent_name);
                
                


                // publish the one with lowest cost or all ?
                int cheapestTask = 0;
                int loopTask = 0;
                double minCost = 1e50;
                for( const auction_msgs::task_allocated& t : tasks_allocated)
                {
                    if(associatedCosts[loopTask] < minCost)
                    {
                        cheapestTask = loopTask;
                        minCost = associatedCosts[loopTask];
                    }
                    loopTask += 1;
                }
                auctionWinners_pub.publish(tasks_allocated[cheapestTask]);
            }


            /*int ttt = 0; // FIX (if no tasks, handle if array is empty or make sure there is allways a -1 if no tasks?)
            if(winners[i][ttt] > -0.5)
            {
                // fix so that allocated winners are saved... TODO
                // if allocated tasks cannot be reallocated, do we remove them from the queue? probably TODO
                auction_msgs::task_allocated task_allocated;

                task_allocated.task = auctionForAssigning.tasks.tasks[winners[i][ttt]];
                task_allocated.agent_ID = bidsCurrentAuction[i].agent_ID;
                task_allocated.agent_name = bidsCurrentAuction[i].agent_name;
                task_allocated.time_allocated = ros::Time::now();

                auctionWinners_pub.publish(task_allocated);




                std::list<auction_ns::Auction_server::task_allocated>::iterator it;
                auction_ns::Auction_server::task_allocated t_win;
                t_win.task = auctionForAssigning.tasks.tasks[winners[i][ttt]];
                it = std::find(taskQueue.begin(), taskQueue.end(), t_win);



                (*it).allocated = true;
                it->allocatedTo = bidsCurrentAuction[i].agent_name;

                agentsThatAreAllocated.push_back(bidsCurrentAuction[i].agent_name);
                


                // TODO: check here if this task has been finished while waitng?
            }*/
        }
        

        //check if the agents ( with task that changed client) are assigned to another task, if not publish "assigned to no task"
        for(const std::string& n : agentsThatWasAllocated)
        {
            bool assignedToAnotherTask = false;
            for(const std::string& assigned : agentsThatAreAllocated)
            {
                if(n == assigned)
                {
                    assignedToAnotherTask = true;
                    break;
                }
            }

            if(assignedToAnotherTask == false)
            {
                // publish something
                std_msgs::String str;
                str.data = n;
                taskNotAllocated_pub.publish(str);
            }
        }





        //clear all bids
        bidsCurrentAuction.erase(bidsCurrentAuction.begin(), bidsCurrentAuction.end());




        std::cout << "currently task queue status: " << std::endl;
        for(const auction_ns::Auction_server::task_allocated& t : taskQueue)
        {
            
            if(t.allocated == true)
            {
                std::cout << "  task (reward: " << t.task.reward << "): " << t.task.task_name << ", " << t.task.task_data << ". ";
                std::cout << "Allocated to: " << t.allocatedTo << std::endl;
            }
            else
            {
                //std::cout << "Not allocated." << std::endl;
            }
        }


        return;
    }




} // namespace auction
