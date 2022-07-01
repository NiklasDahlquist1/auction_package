#include "auction_server.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auction_server");

    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");


    double auctionRoundTime;
    double serverUpdateRate;
    int maxNumberOfBids;
    int tasksAllocatedPerAgent;
    if (!nh_private.getParam ("auctionRoundTime", auctionRoundTime))
        auctionRoundTime = 1;
    if (!nh_private.getParam ("serverUpdateRate", serverUpdateRate))
        serverUpdateRate = 100;
    if (!nh_private.getParam ("tasksAllocatedPerAgent", tasksAllocatedPerAgent))
        tasksAllocatedPerAgent = 2;
    if (!nh_private.getParam ("maxNumberOfBids", maxNumberOfBids))
        maxNumberOfBids = 2;
 

    std::cout << "Server parameters: \n" << "\tauctionRoundTime: " << auctionRoundTime << "\n\tserverUpdateRate: " << serverUpdateRate 
              << "\n\ttasksAllocatedPerAgent: " << tasksAllocatedPerAgent << "\n\tmaxNumberOfBids: " << maxNumberOfBids << std::endl;



    auction_ns::Auction_server auction_server = auction_ns::Auction_server();
    auction_server.initServerParameters(auctionRoundTime, tasksAllocatedPerAgent, maxNumberOfBids);
    

    auction_server.spin_loop(serverUpdateRate);

    return 0;
}





