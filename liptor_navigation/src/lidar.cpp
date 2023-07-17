# include <ros/ros.h>
# include <sensor_msgs/LaserScan.h>
# include <geometry_msgs/Twist.h>

void LidarCallback(const sensor_msgs::LaserScan msg)
{
    float fMidDist = msg.ranges[180];
    ROS_INFO("DISTANCE ranges[180] = %f m", fMidDist);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_node");

    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, &LidarCallback);
    
    ros::spin();

    return 0;
}