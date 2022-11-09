

#include "auction_client.hpp"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "auction_client");

    ros::NodeHandle nh;

    //auction_ns::Auction_client client = auction_ns::Auction_client();
    auction_ns::Auction_client client = auction_ns::Auction_client();



    client.spin_loop(100);



    return 0;
}



