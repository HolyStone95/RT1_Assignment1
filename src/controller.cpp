#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "bot_simulation/Goal.h"

///Global publisher since it will be used in the callback
ros::Publisher pub;
///Global service client since it will be used in the callback
ros::ServiceClient client;
///A costant parameter used in velocity computation
float k=3.0;
///x value for the target position
float goal_x=0.0;
///y value for the target position
float goal_y=0.0;
///variable for the distance
float distance=0.0;
///x value for the robot position
float robot_x;
///y value for the robot position
float robot_y;

///Callback called by subscriber to topic /odom
///
///This Callback receives the robot pose from /odom, computes the distance from the goal.
///If the goal is reached a new target is asked by a client.
///Robot velocity is adjusted with respect to distance.
///@param msg Pointer to a nav_msgs/Odometry message
void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_x=msg->pose.pose.position.x;
  robot_y=msg->pose.pose.position.y;
  distance=sqrt(pow(goal_x-robot_x,2.0)+pow(goal_y-robot_y,2.0));

  geometry_msgs::Twist vel;

  ROS_INFO("Robot position (x , y): [%f , %f]", robot_x, robot_y);

  if( distance <= 0.1 ){
	ROS_INFO("Goal reached!! calculating new objective");
	
	bot_simulation::Goal rec_goal;
	rec_goal.request.min=-6.0;
	rec_goal.request.max=6.0;
	client.call(rec_goal);
	goal_x=rec_goal.response.x;
	goal_y=rec_goal.response.y;
	ROS_INFO("New target position (x , y):[%f , %f]", goal_x, goal_y);
  }
  vel.linear.x= k*(goal_x-robot_x);
  vel.linear.y= k*(goal_y-robot_y);
  pub.publish(vel);

}

///Main controller function 
///
///This node is the actual controller of the robot, it keeps track of position, sets velocities, and asks for new targets.
///Node initialization; Initialiazes a client to a service /goal for receiving a new target.
///Initializes a subscriber to /odom to retrieve robot pose, and a publisher to /cmd_vel to set robot velocity.
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "bot_controller");
  ros::NodeHandle n;

  client = n.serviceClient<bot_simulation::Goal>("/goal");;
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/odom", 1000, positionCallback);

  ros::spin();

  return 0;

}
