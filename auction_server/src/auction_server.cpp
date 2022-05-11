

#include "auction_server.hpp"
#include "optimization_ortools.h"


namespace auction_ns
{
    Auction_server::Auction_server()
    {
        auctionAvailable_pub = nodeHandle.advertise<auction_msgs::auction>("auctionAvailable", 1000);
        auctionWinners_pub = nodeHandle.advertise<auction_msgs::task_allocated>("allocatedTasks", 1000);



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
            if(activeAuction.header.stamp.toSec() + auctionRoundTime <= currentTime) //TODO
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



        std::cout << "tasks in queue:" << std::endl;
        for(auto t : taskQueue)
        {
            std::cout << "    task name: " << t.task.task_name << std::endl;
        }
    }

    void Auction_server::taskFinishedCB(const auction_msgs::task& msg)
    {
        task_allocated taskToRemove;
        bool removeFound = false;
        for(task_allocated t : taskQueue)
        {
            //maybe we should check who sent this?

            if(t.task == msg)
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

        auction_msgs::auction auctionForAssigning;
        auctionForAssigning.auction_ID = activeAuction.auction_ID;
        auctionForAssigning.header = activeAuction.header;
        for(task_allocated t : taskQueue)
        {
            // TODO, filter out some tasks depending on if the current owner has responded or not, if it can be reassign or not, etc.
            auctionForAssigning.tasks.tasks.push_back(t.task);
        }

        // create cost matrix (to allow old optimization function TODO, maybe send the data in a better format?)
        int workers_num = bidsCurrentAuction.size();
        int tasks_num = auctionForAssigning.tasks.tasks.size();
        //std::cout << "workers: " << workers_num << " tasks: " << tasks_num << std::endl;
        std::vector<std::vector<double>> costs(workers_num, std::vector<double>(tasks_num)); //TODO check if every bid is from a unique worker






        // TODO: check here if this task has been finished while waitng. if so, ignore bids for it

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


        std::vector<double> winners(workers_num);
        winners = operations_research::taskMatching(costs); //use the matching optimization




        /*for(int i = 0; i < winners.size(); ++i)
        {
            std::cout << winners[i] << " ";
        }
            std::cout << std::endl;
*/

        std::cout << "Auction round ended, " << bidsCurrentAuction.size() << " bids received" << std::endl;


        for(int i = 0; i < winners.size(); ++i)
        {
            if(winners[i] > -0.5)
            {
                // fix so that allocated winners are saved... TODO
                // if allocated tasks cannot be reallocated, do we remove them from the queue? probably TODO
                auction_msgs::task_allocated task_allocated;

                task_allocated.task = auctionForAssigning.tasks.tasks[winners[i]];
                task_allocated.agent_ID = bidsCurrentAuction[i].agent_ID;
                task_allocated.agent_name = bidsCurrentAuction[i].agent_name;
                task_allocated.time_allocated = ros::Time::now();

                auctionWinners_pub.publish(task_allocated);
                // TODO: check here if this task has been finished while waitng?
            }
        }

        //clear all bids
        bidsCurrentAuction.erase(bidsCurrentAuction.begin(), bidsCurrentAuction.end());




        return;
    }




} // namespace auction
