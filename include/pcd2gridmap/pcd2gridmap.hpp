/*
Pcd2Gridmap (Author: LogWat)
Convert a .pcd file to a gridmap and publish it as a nav_msgs::OccupancyGrid message.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Pcd2Gridmap
{
private:
    ros::NodeHandle nh, pnh;
    ros::Publisher gridmap_pub, metadata_pub;
    
    // Options
    double resolution;
    double min_z, max_z;
    std::string frame_id;
    std::string pcd_file;
    std::string map_topic;
    
    void load_pcd(const std::string &file, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void cloud2gridmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::OccupancyGrid &gridmap);
    void publish_gridmap(const nav_msgs::OccupancyGrid &gridmap);

public:
    Pcd2Gridmap();
    ~Pcd2Gridmap() {}
    void run();
};
