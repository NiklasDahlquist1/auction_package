


#ifndef AUCTION_CLIENT_HPP
#define AUCTION_CLIENT_HPP



#include "ros/ros.h"

#include "auction_msgs/auction.h"
#include "auction_msgs/bid.h"
#include "auction_msgs/price_bid.h"
#include "auction_msgs/task.h"
#include "auction_msgs/taskArray.h"
#include "auction_msgs/task_allocated.h"
#include "auction_msgs/task_finished.h"
#include "auction_msgs/task_result.h"


#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"







namespace auction_ns
{




    class Auction_client
    {
    private:
        bool checkIfIdNotIgnored(int ID);
        void ignoreCurrentTask(); // add current task ID to list of ignore IDs




    protected:
        ros::NodeHandle nodeHandle;

        ros::Publisher auction_bid_pub;
        ros::Publisher auctionFinished_pub;
        ros::Publisher taskResult_pub;
        //ros::Publisher auction_taskAvailable_pub;


        // callbacks
        void checkAvailableAuctionCB(const auction_msgs::auction& msg);
        void taskAllocatedCB(const auction_msgs::task_allocated& msg);
        void taskAlreadyFinishedCB(const auction_msgs::task_allocated& msg); // servers sends finished tasks here, check so that the current task is not already finished
        void taskNoLongerAllocatedCB(const std_msgs::String& msg); // if client was allocated to a task and is now not allocated to any task, servers sends here


        
        ros::Subscriber availAbleAuctions_sub;
        ros::Subscriber taskAllocated_sub;
        ros::Subscriber taskAlreadyFinished_sub;
        ros::Subscriber taskNoLongerAllocated_sub;



        std::vector<int> ignoreIDs;    
        std::string name;
        int client_ID;

        

        auction_msgs::auction currentAuction;
        auction_msgs::task_allocated currentTask;


        void publishTaskResultCurrentTask(std::string result);

    
        bool taskIsActive = false; // client has been allocated a task
        bool taskIsCompleted = false;
        bool auctionAvailableHasBeenSet = false;
        bool auctionIsNew = false; // to know if an auction is new (no bid sent)
        bool taskIsNew = true; // switch behavior when a new task is allocated
        bool bidSent = false; //check to not spam bids

        bool reloadNoTask = true;

        bool isAllocatedToAnyTask = false;

        enum taskStatus {
            COMPLETED,
            WORKING,
            FAILED
        };


        void taskComplete(bool publishConfirmation);
        void taskAssigned(const auction_msgs::task_allocated& msg);

        void createAndSendBid(auction_msgs::auction auction);


        virtual void costForTasks(const std::vector<auction_msgs::task>& tasks, std::vector<auction_msgs::price_bid>& pricesToFill);
        //virtual double costForTask(auction_msgs::task task);
        virtual void handleNewTask(auction_msgs::task task);
        virtual void handleNoTask();
        virtual void executeCurrentBehavior();
        virtual taskStatus currentTaskStatus();

    public:
        Auction_client();
        ~Auction_client();


        //void sendBid(std::vector<double> prices);
        void updateLogic();

        void spin_loop(double loop_rate);



        //void sellCurrentTask(); // put current task up for auctioning again, remember to also sets taskActive to false

        //bt_project::auction_winner getCurrentTask();
    };












} // end namespace


























#endif