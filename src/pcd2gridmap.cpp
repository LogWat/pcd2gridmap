/*
Pcd2Gridmap (Author: LogWat)
Convert a .pcd file to a gridmap and publish it as a nav_msgs::OccupancyGrid message.
The set of points in the specified z-coordinate range (min_z~max_z)
into a 2D map with the specified resolution.
*/

#include "pcd2gridmap/pcd2gridmap.hpp"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>

Pcd2Gridmap::Pcd2Gridmap(): pnh("~")
{
    // Load parameters
    pnh.param<double>("resolution", resolution, 0.1);
    pnh.param<double>("min_z", min_z, 0.0);
    pnh.param<double>("max_z", max_z, 1.0);
    pnh.param<std::string>("frame_id", frame_id, "map");
    pnh.param<std::string>("pcd_file", pcd_file, "test.pcd");

    // Publisher
    gridmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("gridmap", 1);
}

void Pcd2Gridmap::load_pcd(const std::string &file, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) {
        ROS_ERROR("Failed to load %s", file.c_str());
        ros::shutdown();
    }
}

void Pcd2Gridmap::cloud2gridmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::OccupancyGrid &gridmap) {
    // Voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);

    // Get the size of the gridmap
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    for (const auto &p : cloud_filtered->points) {
        min_x = std::min(min_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_x = std::max(max_x, static_cast<double>(p.x));
        max_y = std::max(max_y, static_cast<double>(p.y));
    }
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    // Initialize the gridmap
    gridmap.header.frame_id = frame_id;
    gridmap.info.resolution = resolution;
    gridmap.info.width = width;
    gridmap.info.height = height;
    gridmap.info.origin.position.x = min_x;
    gridmap.info.origin.position.y = min_y;
    gridmap.info.origin.position.z = 0.0;
    gridmap.info.origin.orientation.x = 0.0;
    gridmap.info.origin.orientation.y = 0.0;
    gridmap.info.origin.orientation.z = 0.0;
    gridmap.info.origin.orientation.w = 1.0;
    gridmap.data.resize(width * height, -1);

    // Fill the gridmap
    for (const auto &p : cloud_filtered->points) {
        int x = static_cast<int>((p.x - min_x) / resolution);
        int y = static_cast<int>((p.y - min_y) / resolution);
        int i = x + y * width;
        if (p.z >= min_z && p.z <= max_z) {
            gridmap.data[i] = 100;
        } else {
            if (gridmap.data[i] == -1) gridmap.data[i] = 0;
        }
    }

    // Debug
    ROS_INFO("Gridmap size: %d x %d", width, height);
}

void Pcd2Gridmap::publish_gridmap(const nav_msgs::OccupancyGrid &gridmap) {
    gridmap_pub.publish(gridmap);
}

void Pcd2Gridmap::run() {
    // Load a .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    load_pcd(pcd_file, cloud);

    // Convert the point cloud to a gridmap
    nav_msgs::OccupancyGrid gridmap;
    cloud2gridmap(cloud, gridmap);

    ros::Rate rate(1);
    while (ros::ok()) {
        publish_gridmap(gridmap);
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcd2gridmap");
    Pcd2Gridmap p2g;
    p2g.run();
    return 0;
}
