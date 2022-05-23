

#include "auction_client.hpp"




namespace auction_ns
{
    Auction_client::Auction_client()
    {
        std::cout << "Auction_client" << std::endl;
        name = ros::this_node::getName();
        client_ID = 1337;
        //std::cout << "Client started, node: " << name << std::endl;

        //TODO, set ID here (somehow, maybe get from auction server?) not used for now?

        
        auction_bid_pub = nodeHandle.advertise<auction_msgs::bid>("/submitBid", 1000);
        auctionFinished_pub = nodeHandle.advertise<auction_msgs::task_allocated>("/confirmTaskFinished", 1000);
        taskResult_pub = nodeHandle.advertise<auction_msgs::task_result>("/taskResult", 1000);

        availAbleAuctions_sub = nodeHandle.subscribe("/auctionAvailable", 100, &Auction_client::checkAvailableAuctionCB, this);
        taskAllocated_sub = nodeHandle.subscribe("/allocatedTasks", 100, &Auction_client::taskAllocatedCB, this);
        taskAlreadyFinished_sub = nodeHandle.subscribe("/confirmTaskFinished", 100, &Auction_client::taskAlreadyFinishedCB, this);


    }
    Auction_client::~Auction_client()
    {
        std::cout << "Client stopped, node: " << name << std::endl;
    }

    void Auction_client::createAndSendBid(auction_msgs::auction auction)
    {
        auction_msgs::bid bid;

        bid.agent_name = name;
        bid.agent_ID = client_ID;
        bid.auction_ID = auction.auction_ID;


        costForTasks(auction.tasks.tasks, bid.prices);

/*
        for(auction_msgs::task t : auction.tasks.tasks)
        {
            auction_msgs::price_bid p;
            p.task_ID = t.task_ID;
            p.price = costForTask(t);
            
            bid.prices.push_back(p);
        }
*/
        auction_bid_pub.publish(bid);
    }

    // virtual functions...
    void Auction_client::handleNewTask(auction_msgs::task task)
    {
        std::cout << "handle new task not implemented" << std::endl;
    }

    void Auction_client::handleNoTask()
    {
        std::cout << "handle no task not implemented" << std::endl;
    }
    void Auction_client::executeCurrentBehavior()
    {
        std::cout << "execute current behavior not implemented" << std::endl;
    }
/*    double Auction_client::costForTask(auction_msgs::task task)
    {
        std::cout << "cost for task function not implemented" << std::endl;
        return -1;
    }*/
    void Auction_client::costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill)
    {
        return;
    }
    Auction_client::taskStatus Auction_client::currentTaskStatus()
    {
        std::cout << "task is completed function not implemented" << std::endl;
        return Auction_client::WORKING;
    }

    void Auction_client::updateLogic()
    {
        //std::cout << "updateLogic" << std::endl;
        if(auctionIsNew == true) // TODO, add so that if a task cannot be reallocated the client should not place new bids (AND taskCanBeReallocated or something)
        {
            // send bid
            createAndSendBid(currentAuction);
            auctionIsNew = false;
        }

        if(taskIsActive)
        {
            if(taskIsNew)
            {
                //switch behavior
                handleNewTask(currentTask.task);
                taskIsNew = false;
            }
            // execute behavior
            executeCurrentBehavior();

            Auction_client::taskStatus statusTask = currentTaskStatus();

            if(statusTask == Auction_client::COMPLETED)
            {
                taskIsCompleted = true;
            }
            else if(statusTask == Auction_client::WORKING)
            {
                // continue executing?
            }
            else if(statusTask == Auction_client::FAILED)
            {
                // maybe return task (and not bid on it again as previous implementation)?
            }
        }
        else if(taskIsActive == false)
        {
            if(reloadNoTask)
            {
                handleNoTask();
                reloadNoTask = false;
            }
            // switch to default behavior
            // execute behavior
            executeCurrentBehavior();
        }

        if(taskIsCompleted == true)
        {
            taskComplete(true);
        }
    }

    void Auction_client::taskComplete(bool publishConfirmation)
    {
        if(publishConfirmation == true)
        {
            auctionFinished_pub.publish(currentTask); // confirm to the auction server that the current task is completed
        }
        taskIsActive = false;
        //taskIsNew = true; // switch behavior to noTask()
        reloadNoTask = true;
        taskIsCompleted = false;
    }


    // call to publish result related to the current task
    void Auction_client::publishTaskResultCurrentTask(std::string result)
    {
        auction_msgs::task_result task_res;
        task_res.task = currentTask.task;
        task_res.agent_ID_result = client_ID;
        task_res.agent_name_result = name;
        task_res.result_time = ros::Time::now();
        task_res.result = result;

        taskResult_pub.publish(task_res);
    }


    void Auction_client::taskAssigned(const auction_msgs::task_allocated& msg)
    {
        // check if a new task is allocated
        if(currentTask.task != msg.task)
        {
            taskIsNew = true; 
        }

        currentTask = msg;
        //set this task as active
        taskIsActive = true;
        taskIsCompleted = false;

        isAllocatedToAnyTask = true;
    }



    void Auction_client::spin_loop(double loop_rate)
    {
        std::cout << "SPINLOOP" << std::endl;
        ros::Rate rate = ros::Rate(loop_rate);
        while (ros::ok())
        {
            ros::spinOnce();

            updateLogic();
            rate.sleep();
        }
    }




    void Auction_client::checkAvailableAuctionCB(const auction_msgs::auction& msg)
    {
        currentAuction = msg;

        // a auction is now available
        auctionAvailableHasBeenSet = true; // TODO, hmm
        bidSent = false;
        auctionIsNew = true;
    }


    void Auction_client::taskAllocatedCB(const auction_msgs::task_allocated& msg)
    {
        bool isAllocatedToAnyTask = false;
        //check if the name of the allocated client is this one
        if(name == msg.agent_name)
        {
            taskAssigned(msg);
        }
        else if (msg.task == currentTask.task) // someone else is assigned the current task
        {
            // maybe set current task to nothing? TODO

            // task is "complete", switch behavior
            taskIsActive = false;
            reloadNoTask = true;
            taskIsCompleted = false;
        }
    }
        



    void Auction_client::taskAlreadyFinishedCB(const auction_msgs::task_allocated& msg)
    {
        //std::cout << "test: RECIEVED TASK FINISNNNG" << std::endl;

        // no current task, return
        if(taskIsActive == false)
        {
            return;
        }

        // someone else finished current task
        if(msg.task == currentTask.task)
        {
            taskComplete(false); // no need to publish, task already finished
        }
    }












} // namespace auction_ns












