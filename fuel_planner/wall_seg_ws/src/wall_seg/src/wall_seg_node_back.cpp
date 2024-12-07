#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PCLPointCloud2.h> // Include for fieldList
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/region_growing.h> 
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

using namespace pcl;


struct obstacle_wall {
    // 墙ID
    uint8_t id;
    // 墙内点的xyz最大值与最小值
    PointXYZ max;
    PointXYZ min;
    // 墙的中心和法向量
    PointNormal normal;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_seg_node");
    ros::NodeHandle nh("~");
    ros::Publisher pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("colored_clusters_output", 1);
    ros::Publisher pub_normals = nh.advertise<visualization_msgs::MarkerArray>("normals", 10);
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

    //加载点云文件
    if (pcl::io::loadPCDFile("/home/egan/wall_seg_ws/src/wall_seg/pcd/无标题2.pcd", *cloud) == -1) {
        ROS_ERROR("NiaYiGoJi");
        return -1;
    }


    NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud(cloud);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree < PointXYZ >());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(0.1);

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
    ne.compute(*cloud_normals);

    

    
    // 将XYZ和法向量合并到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);

    // 创建区域生长聚类对象
    RegionGrowing<PointNormal, Normal> reg;
    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(1000000);

    search::KdTree<PointNormal>::Ptr tree2(new search::KdTree < PointNormal >());

    reg.setSearchMethod(tree2);
    reg.setNumberOfNeighbours(20);
    reg.setInputCloud(cloud_with_normals);
    reg.setInputNormals(cloud_normals);
    reg.setSmoothnessThreshold(2.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector <pcl::PointIndices> clusters;
    // 执行聚类
    reg.extract(clusters);

    ROS_INFO("clusters count: %lu", clusters.size());

    // 创建一个点云来存储带有颜色的聚类结果
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->points.resize(cloud->points.size());

    // 为每个簇分配颜色
    for (int i = 0; i < clusters.size(); ++i)
    {
        uint8_t r = rand() % 255;
        uint8_t g = rand() % 255;
        uint8_t b = rand() % 255;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

        for (int j = 0; j < clusters[i].indices.size(); ++j)
        {
            int index = clusters[i].indices[j];
            colored_cloud->points[index].x = cloud->points[index].x;
            colored_cloud->points[index].y = cloud->points[index].y;
            colored_cloud->points[index].z = cloud->points[index].z;
            colored_cloud->points[index].rgb = *reinterpret_cast<float*>(&rgb);
        }
    }


    // 创建墙列表
    std::vector<obstacle_wall> walls;
    for (int i = 0; i < clusters.size(); ++i) {
        obstacle_wall* wall = new obstacle_wall();

        float maxX = cloud_with_normals->points[clusters[i].indices[0]].x;
        float maxY = cloud_with_normals->points[clusters[i].indices[0]].y;
        float maxZ = cloud_with_normals->points[clusters[i].indices[0]].z;
        float minX = cloud_with_normals->points[clusters[i].indices[0]].x;
        float minY = cloud_with_normals->points[clusters[i].indices[0]].y;
        float minZ = cloud_with_normals->points[clusters[i].indices[0]].z;


        PointXYZ center;
        for (int j = 0;j < clusters[i].indices.size();j++) {
            auto index = clusters[i].indices[j];
            wall->normal.normal_x += cloud_with_normals->points[index].normal_x;
            wall->normal.normal_y += cloud_with_normals->points[index].normal_y;
            wall->normal.normal_z += cloud_with_normals->points[index].normal_z;
            maxX = std::max(maxX, cloud_with_normals->points[index].x);
            maxY = std::max(maxY, cloud_with_normals->points[index].y);
            maxZ = std::max(maxZ, cloud_with_normals->points[index].z);
            minX = std::min(minX, cloud_with_normals->points[index].x);
            minY = std::min(minY, cloud_with_normals->points[index].y);
            minZ = std::min(minZ, cloud_with_normals->points[index].z);

            wall->normal.x += cloud_with_normals->points[index].x;
            wall->normal.y += cloud_with_normals->points[index].y;
            wall->normal.z += cloud_with_normals->points[index].z;
        }
        wall->normal.normal_x /= clusters[i].indices.size();
        wall->normal.normal_y /= clusters[i].indices.size();
        wall->normal.normal_z /= clusters[i].indices.size();
        wall->normal.x /= clusters[i].indices.size();
        wall->normal.y /= clusters[i].indices.size();
        wall->normal.z /= clusters[i].indices.size();

        wall->max.x = maxX;
        wall->max.y = maxY;
        wall->max.z = maxZ;
        wall->min.x = minX;
        wall->min.y = minY;
        wall->min.z = minZ;

        wall->id = i;

        walls.push_back(*wall);
    }


    // 发布聚类后的点云
    sensor_msgs::PointCloud2 output_clusters;
    pcl::toROSMsg(*colored_cloud, output_clusters);
    output_clusters.header.frame_id = "map";

    // 发布墙信息
    // 在墙中心点画出法向量
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "map";
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.ns = "normal";
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.scale.x = 0.5; // 箭头的大小
    arrow_marker.scale.y = 0.1;
    arrow_marker.scale.z = 0.1;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.0;
    // 创建一个箭头的Marker
    for (int i = 0; i < walls.size(); ++i) {
        arrow_marker.id = walls[i].id;
        arrow_marker.pose.position.x = walls[i].normal.x;
        arrow_marker.pose.position.y = walls[i].normal.y;
        arrow_marker.pose.position.z = walls[i].normal.z;

        Eigen::Vector3f normal = walls[i].normal.getNormalVector3fMap();

        normal.normalize();  // 归一化法向量
        Eigen::Vector3f y_axis(1.0, 0.0, 0.0);
        Eigen::Quaternionf q;

        if (normal == y_axis) {
            q = Eigen::Quaternionf::Identity();
        }
        else {
            Eigen::Vector3f axis = y_axis.cross(normal).normalized();
            double angle = acos(y_axis.dot(normal));
            q = Eigen::AngleAxisf(angle, axis);
        }
        arrow_marker.pose.orientation.w = q.w();
        arrow_marker.pose.orientation.x = q.x();
        arrow_marker.pose.orientation.y = q.y();
        arrow_marker.pose.orientation.z = q.z();

        marker_array.markers.push_back(arrow_marker);
    }

    ros::Rate loop_rate(1);
    while (nh.ok())
    {
        pub_clusters.publish(output_clusters);
        pub_normals.publish(marker_array);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}