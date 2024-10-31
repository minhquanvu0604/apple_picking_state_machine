#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    // Publisher for the point cloud
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("saved_pointcloud", 1);

    std::string yaml_file = ros::package::getPath("apple_picking_state_machine") + "/config/components_coordination_config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_file);
    std::string pcd_file_path = config["pcd_publisher"]["file_path"].as<std::string>();   
    ROS_INFO("Loading PCD file from %s", pcd_file_path.c_str());

    // Load PCD file with color
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file_path, cloud) == -1) {
        ROS_ERROR("Couldn't read the PCD file with color.");
        return -1;
    }

    // Convert to ROS message
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "world";  // Change the frame if necessary

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        output.header.stamp = ros::Time::now();
        pcd_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
