#include "auction_server.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auction_server");

    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");


    double auctionRoundTime;
    double serverUpdateRate;
    if (!nh_private.getParam ("auctionRoundTime", auctionRoundTime))
        auctionRoundTime = 1;
    if (!nh_private.getParam ("serverUpdateRate", serverUpdateRate))
        serverUpdateRate = 200;


        

    std::cout << "Using auctionRoundTime = " << auctionRoundTime << " and serverUpdateRate = " << serverUpdateRate << std::endl;



    auction_ns::Auction_server auction_server = auction_ns::Auction_server();
    auction_server.initServerParameters(auctionRoundTime);
    

    auction_server.spin_loop(serverUpdateRate);

    return 0;
}





