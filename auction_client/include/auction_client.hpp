


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
        //ros::Publisher auction_taskAvailable_pub;


        // callbacks
        void checkAvailableAuctionCB(const auction_msgs::auction& msg);
        void taskAllocatedCB(const auction_msgs::task_allocated& msg);
        
        ros::Subscriber availAbleAuctions_sub;
        ros::Subscriber taskAllocated_sub;



        std::vector<int> ignoreIDs;    
        std::string name;

        

        auction_msgs::auction currentAuction;
        auction_msgs::task_allocated currentTask;



 
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



        void createAndSendBid(auction_msgs::auction auction);


        virtual double costForTask(auction_msgs::task task);
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




/*
    class Auction_client_bt : public Auction_client
    {
        private:

        //BT
        //factory, etc

        // splitString()

        // callbacks
        void poseCB(const geometry_msgs::Pose& msg);

        ros::Subscriber currentPose_sub;


        void handleNewTask(auction_msgs::task task); // swtitch to bt for task, initialize new tree nodes, etc.
        void handleNoTask(); // switch to no task bt
        void executeCurrentBehavior(); // tick bt, also check if task is completed
        taskStatus currentTaskStatus(); // check if task is completed, (maybe return bt status? in this case)

        protected:

        double costForTask(auction_msgs::task task); // return cost for task, (distance, cost from dsp, time, etc.)
        // generate BT, initialize nodes etc.
        // generate no task BT, init nodes...
        // 

        public:
        using Auction_client::Auction_client;

        Auction_client_bt(); // setup callbacks, state (used for bt), etc. here

    };
*/


    class Auction_client_uav : public Auction_client
    {
        private:

        geometry_msgs::Pose currentPose; // current pose needed to calculate distance for cost function, could be implemented as a service i.e asked for when needed?

        // callbacks
        void poseCB(const geometry_msgs::Pose& msg);

        ros::Subscriber currentPose_sub;

        protected:

        double costForTask(auction_msgs::task task);
        void handleNewTask(auction_msgs::task task);
        void handleNoTask();
        void executeCurrentBehavior();

        public:
        using Auction_client::Auction_client;

        Auction_client_uav();

    };



    //TODO, implement client for for other types of agents












} // end namespace


























#endif