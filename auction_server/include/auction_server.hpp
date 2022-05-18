



#ifndef AUCTION_SERVER_HPP
#define AUCTION_SERVER_HPP



#include "ros/ros.h"

#include "auction_msgs/auction.h"
#include "auction_msgs/bid.h"
#include "auction_msgs/price_bid.h"
#include "auction_msgs/task.h"
#include "auction_msgs/taskArray.h"
#include "auction_msgs/task_allocated.h"
#include "auction_msgs/task_finished.h"

#include "std_msgs/String.h"


// TODO: add ortools, but I dont like to just add the folder...
//#include "optimization_ortools.h"


namespace auction_ns
{
    class Auction_server
    {
        private:
        struct task_allocated
        {
            auction_msgs::task task;
            bool allocated = false;
            std::string allocatedTo;



            bool operator==(const task_allocated& t) const // hmm, maybe not great?
            {
                return t.task == task;
            }
        };



        ros::NodeHandle nodeHandle;
        ros::NodeHandle nodeHandle_private;

        ros::Publisher auctionAvailable_pub;
        ros::Publisher auctionWinners_pub;
        ros::Publisher taskFinished_pub;
        //ros::Publisher auction_return_pub;

        int auctionIDCounter = 0; // count ID for published auctions
        bool auctionRunning = false;



        std::list<task_allocated> taskQueue; // all available tasks
        auction_msgs::auction activeAuction;

        std::vector<auction_msgs::bid> bidsCurrentAuction;

        std::vector<std::string> clientsRegistered;
        //std::list<auction_msgs::task_orderedList> t;


        //parameters?

        double auctionRoundTime;



        // callbacks
        void bidsCB(const auction_msgs::bid& msg);
        void addTasksCB(const auction_msgs::taskArray& msg);
        //void newOrderedListTasksVB() // used for adding a list of tasks to be finished in a sequence, offer the next when one is reported finished
        void taskFinishedCB(const auction_msgs::task_allocated& msg);
        void newClientCB(const std_msgs::String& msg); // maybe use these clients to check things (like if they have submitted a bid or not?)
        //TODO, add callback to register clients. Also, add a way to end an auction if all clients have submitted (also a flag to enable/disable)

        // subs
        ros::Subscriber bids_sub;
        ros::Subscriber addTasks_sub;
        ros::Subscriber taskFinished_sub;
        ros::Subscriber newClient_sub;




        //
        void createAndPublishNewAuction();
        void declareWinner();

        // functions

        // remove task based on ID


        public:
        Auction_server();
        ~Auction_server();

        void initServerParameters(double auctionRoundTime);

        void updateLogic();
        void spin_loop(double loop_rate);


        // request ID (rosservice? or use random number? seed for random generator?)



    };




} // end namespace auction





/* 
logic:

if waiting for bids:
    check if auction is finished, if so:
        optimize winners (for each task: check if current "owner" has submitted a bid, if not. do not offer it (if can_be_reallocated_no_respons == true))
        allocate tasks (dont keep track of (can_be_abandoned == false) tasks, handle as old auction_server)

else if(tasks available)
    create list of current tasks for auction 
    (exclude tasks that can not be reallocated etc)
    publish tasks (start auction round)


CBs:

task received ()
bid received ()
new client (to check if they have submitted a bid for reallocation)
task finished (remove from task queue, check if finished task belongs to list of oredered tasks, if so: add next task to queue. check if the next task is a "can be abandoned" task, if so give warning, etc?)

*/


#endif