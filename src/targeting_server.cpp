#include "ros/ros.h"
#include "bot_simulation/Goal.h"

///Function to get a random number in a range
///
///Returns a random double in a range between a minimum and a maximum
///@param M minimum
///@param N maximum
double randMToN(double M, double N){
	return M + (rand() / (RAND_MAX / (N-M)));
}

///Service Callback for /goal server
///
///Calls the function randMToN in order to populate the service response based on the request
///@param req service request
///@param res service response
bool myrandom (bot_simulation::Goal::Request &req, bot_simulation::Goal::Response &res){
	res.x = randMToN (req.min, req.max);
	res.y = randMToN (req.min, req.max);

	return true;
}

///Targeting server Node
///
///This node implements the server for /goal service, in charge of computing a new target position
///Node initialization; Server initialization for the service /goal.
int main (int argc, char **argv)
{
	ros::init(argc,argv,"targeting_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/goal", myrandom);	
 	ros::spin();

	return 0; 
}
