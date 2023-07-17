#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

ros::Publisher cmdVelPub;
ros::Publisher marker_pub;

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());
  ros::Duration(1).sleep(); // sleep for  a second
  ROS_INFO("nav_square.cpp ended!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_move_base");
  std::string topic = "/cmd_vel";
  ros::NodeHandle node;
  //Subscribe to the move_base action server
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

  signal(SIGINT, shutdown);
  ROS_INFO("move_base_square.cpp start...");

  //How big is the square we want the robot to navigate?
  double square_size = -2.0;

  //Create a list to hold the target quaternions (orientations)
  geometry_msgs::Quaternion quaternions[4];

  //convert the angles to quaternions
  double angle = 3*M_PI/4;
  int angle_count = 0;
  quaternions[angle_count] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
  //for(angle_count = 0; angle_count < 4;angle_count++ )
  //{
  //    quaternions[angle_count] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
  //    angle = angle + M_PI/2;
  //}

  //a pose consisting of a position and orientation in the map frame.
  geometry_msgs::Point point;
  geometry_msgs::Pose pose_list[0];
  point.x = -8.0;
  point.y = 5.4;
  point.z = 0.0;
  pose_list[0].position = point;
  pose_list[0].orientation = quaternions[0];

  //Wait 60 seconds for the action server to become available
  if(!ac.waitForServer(ros::Duration(60)))
  {
    ROS_INFO("Can't connected to move base server");
    return 1;
  }
  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");

  //Initialize a counter to track waypoints
  int count = 0;

  //Intialize the waypoint goal
  move_base_msgs::MoveBaseGoal goal;

  //Use the map frame to define goal poses
  goal.target_pose.header.frame_id = "map";

  //Set the time stamp to "now"
  goal.target_pose.header.stamp = ros::Time::now();

  //Set the goal pose to the i-th waypoint
  goal.target_pose.pose = pose_list[count];

  //Start the robot moving toward the goal
  //Send the goal pose to the MoveBaseAction server
  ac.sendGoal(goal);

  //Allow 1 minute to get there
  bool finished_within_time = ac.waitForResult(ros::Duration(180));

  //If we dont get there in time, abort the goal
  if(!finished_within_time)
  {
      ac.cancelGoal();
      ROS_INFO("Timed out achieving goal");
  }
  else
  {
      //We made it!
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Goal succeeded!");
      }
      else
      {
        ROS_INFO("The base failed for some reason");
      }

     count += 1;
  }

ROS_INFO("move_base_square.cpp end...");
return 0;
}


