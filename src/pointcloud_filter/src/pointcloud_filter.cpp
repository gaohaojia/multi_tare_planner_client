#include <math.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/statistical_outlier_removal.h>

class pointcloud_filter : public rclcpp::Node
{
    public :
        pointcloud_filter(std::string name): Node(name)
        {
            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_registered", 5, std::bind(&pointcloud_filter::sub_callback, this, std::placeholders::_1));
            pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_filtered", 5);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
        pcl::CropBox<pcl::PointXYZI> box_filter1, box_filter2;
        //pcl::StatisticalOutlierRemoval<pcl::PointXYZI> Static;
        //pcl::RadiusOutlierRemoval<pcl::PointXYZI> sor;

        void sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
    
        box_filter1.setMin(Eigen::Vector4f(-0.3, -0.1, -0.1, 1.0));
        box_filter1.setMax(Eigen::Vector4f(0.0, 0.1, 0.1, 1.0));
        box_filter1.setNegative(true);
        box_filter1.setInputCloud(cloud);
        box_filter1.filter(*cloud);
        
        // box_filter2.setMin(Eigen::Vector4f(-0.6, -0.6, -5.0, 1.0));
        // box_filter2.setMax(Eigen::Vector4f(0.6, 0.6, 5.0, 1.0));
        // box_filter2.setNegative(true);
        // box_filter2.setInputCloud(cloud);
        // box_filter2.filter(*cloud);
        
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZI>);
        // sor.setRadiusSearch(0.1);
        // sor.setMinNeighborsInRadius(2);
        // sor.setNegative(false);
        // sor.setInputCloud(cloud_filtered2);
        // sor.filter(*cloud_filtered3);
        
        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZI>);
        //Static.setMeanK(50);
        //Static.setStddevMulThresh(1.0);
        //Static.setInputCloud(cloud_filtered2);
        //Static.filter(*cloud_filtered3);
        
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud, filtered_msg);
        filtered_msg.header.frame_id = cloud_msg->header.frame_id;
        filtered_msg.header.stamp = now() ;
        cloud_msg->header.stamp;
        filtered_msg.fields = cloud_msg->fields;
        pub->publish(filtered_msg);
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_filter>("pointcloud_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
}