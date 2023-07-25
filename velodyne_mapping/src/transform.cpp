#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

void LidarCallback(const sensor_msgs::PointCloud)
{




}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "transform");
    ros::NodeHandle node;
    ros::Subscriber lidar_sub = node.subscribe("/velodyne_points", 10, &LidarCallback);
    ros::spin();
    return 0;
}

