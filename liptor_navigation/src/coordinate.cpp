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
geometry_msgs::Quaternion quaternions;

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());
  ros::Duration(1).sleep(); // sleep for  a second
  ROS_INFO("nav_square.cpp ended!");
  ros::shutdown();
}

void PointCallback(const geometry_msgs::Point point)
{
  //a pose consisting of a position and orientation in the map frame.
  geometry_msgs::Pose pose_list[0];
  pose_list[0].position.x = point.x;
  pose_list[0].position.y = point.y;
  pose_list[0].position.z = 0;
  pose_list[0].orientation = quaternions;

  //Subscribe to the move_base action server
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

  //Wait 60 seconds for the action server to become available
  if(!ac.waitForServer(ros::Duration(60)))
  {
    ROS_INFO("Can't connected to move base server");
  }
  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");

  //Intialize the waypoint goal
  move_base_msgs::MoveBaseGoal goal;

  //Use the map frame to define goal poses
  goal.target_pose.header.frame_id = "map";

  //Set the time stamp to "now"
  goal.target_pose.header.stamp = ros::Time::now();

  //Set the goal pose to the i-th waypoint
  goal.target_pose.pose = pose_list[0];

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


  }


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinate");
  std::string topic = "/cmd_vel";
  ros::NodeHandle node;


  signal(SIGINT, shutdown);
  ROS_INFO("move_base_square.cpp start...");

  //Create a list to hold the target quaternions (orientations)
  
  geometry_msgs::Point point;

  //convert the angles to quaternions
  double angle = 3*M_PI/4;
  quaternions = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);

  ros::Subscriber point_sub = node.subscribe("/RubbishCoordinate", 10, &PointCallback);
  
  ros::spin();

  return 0;
 
}
