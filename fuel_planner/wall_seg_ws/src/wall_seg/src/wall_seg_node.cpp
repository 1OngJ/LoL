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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/region_growing.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h> // Added for CropBox filter
#include <wall_seg/WallInfo.h>

using namespace pcl;

class obstacle_wall {
public:
    uint8_t id;
    PointXYZ max;
    PointXYZ min;
    PointNormal normal;
    double angle;
    bool explored;
    Eigen::Vector3d centroid;
};

// 无人机当前位置
geometry_msgs::PointStamped cur_position;

ros::Publisher pub_clusters;
ros::Publisher pub_normals;
ros::Publisher pub_wall_info;

double distanceThreshold;
double angleThreshold;
int minClusterSize;
int maxClusterSize;
int numberOfNeighbours;
double smoothnessThreshold;
double radiusSearch;
double interestRadius;


// 计算两平面的夹角，返回值为弧度(0-90)
double calculateAngle(const Eigen::Vector3f& normal1, const Eigen::Vector3f& normal2) {
    Eigen::Vector3f normalized_normal1 = normal1.normalized();
    Eigen::Vector3f normalized_normal2 = normal2.normalized();
    return acos(abs(normalized_normal1.dot(normalized_normal2)));
}

// 判断两个平面是否相邻，判断条件为两平面距离和夹角
bool isAdjacent(const obstacle_wall& wall1, const obstacle_wall& wall2) {
    double distx1 = std::abs(wall1.min.x - wall2.min.x);
    double distx2 = std::abs(wall1.min.x - wall2.max.x);
    double distx3 = std::abs(wall1.max.x - wall2.min.x);
    double distx4 = std::abs(wall1.max.x - wall2.max.x);

    double disty1 = std::abs(wall1.min.y - wall2.min.y);
    double disty2 = std::abs(wall1.min.y - wall2.max.y);
    double disty3 = std::abs(wall1.max.y - wall2.min.y);
    double disty4 = std::abs(wall1.max.y - wall2.max.y);

    double d1 = sqrt(distx1 * distx1 + disty1 * disty1);
    double d2 = sqrt(distx2 * distx2 + disty2 * disty2);
    double d3 = sqrt(distx3 * distx3 + disty3 * disty3);
    double d4 = sqrt(distx4 * distx4 + disty4 * disty4);

    Eigen::Vector3f normal1(wall1.normal.normal_x, wall1.normal.normal_y, wall1.normal.normal_z);
    Eigen::Vector3f normal2(wall2.normal.normal_x, wall2.normal.normal_y, wall2.normal.normal_z);
    double angleRad = calculateAngle(normal1, normal2);
    double angleDeg = angleRad * 180.0 / M_PI;

    return std::min({ d1,d2,d3,d4 }) <= distanceThreshold && angleDeg > angleThreshold;
}


void pointcloudInfoCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 1. Convert ROS PointCloud2 to PCL PointCloud
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    fromROSMsg(*msg, *cloud);

    // 2. Crop the point cloud based on current drone position and radius
    PointCloud<PointXYZ>::Ptr cropped_cloud(new PointCloud<PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> cropFilter;
    cropFilter.setInputCloud(cloud);

    Eigen::Vector4f minPoint;
    minPoint[0] = cur_position.point.x - interestRadius;  // x
    minPoint[1] = cur_position.point.y - interestRadius;  // y
    minPoint[2] = cur_position.point.z - interestRadius;  // z
    cropFilter.setMin(minPoint);

    Eigen::Vector4f maxPoint;
    maxPoint[0] = cur_position.point.x + interestRadius;  // x
    maxPoint[1] = cur_position.point.y + interestRadius;  // y
    maxPoint[2] = cur_position.point.z + interestRadius;  // z
    cropFilter.setMax(maxPoint);


    cropFilter.filter(*cropped_cloud);

    if (cropped_cloud->empty()) {
        ROS_WARN("Cropped point cloud is empty. Skipping processing.");
        return;
    }



    //  以下部分使用cropped_cloud代替cloud
    NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud(cropped_cloud);
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radiusSearch);

    PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>);
    ne.compute(*cloud_normals);


    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cropped_cloud, *cloud_normals, *cloud_with_normals);

    RegionGrowing<PointNormal, Normal> reg;
    reg.setMinClusterSize(minClusterSize);
    reg.setMaxClusterSize(maxClusterSize);

    search::KdTree<PointNormal>::Ptr tree2(new search::KdTree<PointNormal>());

    reg.setSearchMethod(tree2);
    reg.setNumberOfNeighbours(numberOfNeighbours);
    reg.setInputCloud(cloud_with_normals);
    reg.setInputNormals(cloud_normals);
    reg.setSmoothnessThreshold(smoothnessThreshold / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->points.resize(cropped_cloud->points.size());


    for (int i = 0; i < clusters.size(); ++i) {
        uint8_t r = rand() % 255;
        uint8_t g = rand() % 255;
        uint8_t b = rand() % 255;
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

        for (int j = 0; j < clusters[i].indices.size(); ++j) {
            int index = clusters[i].indices[j];
            colored_cloud->points[index].x = cropped_cloud->points[index].x;
            colored_cloud->points[index].y = cropped_cloud->points[index].y;
            colored_cloud->points[index].z = cropped_cloud->points[index].z;
            colored_cloud->points[index].rgb = *reinterpret_cast<float*>(&rgb);
        }
    }


    sensor_msgs::PointCloud2 output_clusters;
    pcl::toROSMsg(*colored_cloud, output_clusters);
    output_clusters.header.frame_id = "world";
    output_clusters.header.stamp = ros::Time::now();
    pub_clusters.publish(output_clusters);


    std::vector<obstacle_wall> walls;
    for (int i = 0; i < clusters.size(); ++i) {
        obstacle_wall* wall = new obstacle_wall();

        wall->id = i;
        wall->angle = 0.0;
        wall->explored = false;

        float maxX = cloud_with_normals->points[clusters[i].indices[0]].x;
        float maxY = cloud_with_normals->points[clusters[i].indices[0]].y;
        float maxZ = cloud_with_normals->points[clusters[i].indices[0]].z;
        float minX = cloud_with_normals->points[clusters[i].indices[0]].x;
        float minY = cloud_with_normals->points[clusters[i].indices[0]].y;
        float minZ = cloud_with_normals->points[clusters[i].indices[0]].z;

        wall->centroid = Eigen::Vector3d::Zero();
        wall->normal.normal_x = wall->normal.normal_y = wall->normal.normal_z = 0;
        wall->normal.x = wall->normal.y = wall->normal.z = 0;

        for (int j = 0; j < clusters[i].indices.size(); j++) {
            auto index = clusters[i].indices[j];

            maxX = std::max(maxX, cloud_with_normals->points[index].x);
            maxY = std::max(maxY, cloud_with_normals->points[index].y);
            maxZ = std::max(maxZ, cloud_with_normals->points[index].z);
            minX = std::min(minX, cloud_with_normals->points[index].x);
            minY = std::min(minY, cloud_with_normals->points[index].y);
            minZ = std::min(minZ, cloud_with_normals->points[index].z);

            wall->normal.normal_x += cloud_with_normals->points[index].normal_x;
            wall->normal.normal_y += cloud_with_normals->points[index].normal_y;
            wall->normal.normal_z += cloud_with_normals->points[index].normal_z;

            wall->normal.x += cloud_with_normals->points[index].x;
            wall->normal.y += cloud_with_normals->points[index].y;
            wall->normal.z += cloud_with_normals->points[index].z;

            wall->centroid += cloud_with_normals->points[index].getVector3fMap().cast<double>();
        }

        wall->centroid /= clusters[i].indices.size();

        wall->normal.normal_x /= clusters[i].indices.size();
        wall->normal.normal_y /= clusters[i].indices.size();
        wall->normal.normal_z /= clusters[i].indices.size();

        wall->normal.x = wall->centroid.x();
        wall->normal.y = wall->centroid.y();
        wall->normal.z = wall->centroid.z();

        wall->max.x = maxX;
        wall->max.y = maxY;
        wall->max.z = maxZ;
        wall->min.x = minX;
        wall->min.y = minY;
        wall->min.z = minZ;


        if (abs(wall->normal.normal_z) > abs(sqrt(wall->normal.normal_x * wall->normal.normal_x + wall->normal.normal_y * wall->normal.normal_y)) / 2) continue;

        walls.push_back(*wall);
    }



    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker arrow_marker;
    visualization_msgs::Marker text_marker;

    arrow_marker.header.frame_id = "world";
    arrow_marker.header.stamp = ros::Time::now();
    arrow_marker.ns = "normal";
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.scale.x = 0.5;
    arrow_marker.scale.y = 0.1;
    arrow_marker.scale.z = 0.1;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.0;


    text_marker.header.frame_id = "world";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "id";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.orientation.w = 1.0;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.6;
    text_marker.color.b = 255;
    text_marker.color.g = 255;
    text_marker.color.r = 0;
    text_marker.color.a = 1;
    for (int i = 0; i < walls.size(); ++i) {
        arrow_marker.id = walls[i].id;
        arrow_marker.pose.position.x = walls[i].normal.x;
        arrow_marker.pose.position.y = walls[i].normal.y;
        arrow_marker.pose.position.z = walls[i].normal.z;

        Eigen::Vector3f normal = walls[i].normal.getNormalVector3fMap();

        normal.normalize();
        Eigen::Vector3f x_axis(1.0, 0.0, 0.0);
        Eigen::Quaternionf q;

        if (normal == x_axis) {
            q = Eigen::Quaternionf::Identity();
        }
        else {
            Eigen::Vector3f axis = x_axis.cross(normal).normalized();
            double angle = acos(x_axis.dot(normal));
            q = Eigen::AngleAxisf(angle, axis);
        }
        arrow_marker.pose.orientation.w = q.w();
        arrow_marker.pose.orientation.x = q.x();
        arrow_marker.pose.orientation.y = q.y();
        arrow_marker.pose.orientation.z = q.z();

        marker_array.markers.push_back(arrow_marker);

        text_marker.id = walls[i].id;
        geometry_msgs::Pose pose;
        pose.position.x = walls[i].normal.x;
        pose.position.y = walls[i].normal.y;
        pose.position.z = walls[i].normal.z + 1;
        std::ostringstream str;
        str << "-" << (int)walls[i].id << "-";
        text_marker.text = str.str();
        text_marker.pose = pose;
        marker_array.markers.push_back(text_marker);

    }
    pub_normals.publish(marker_array);


    for (size_t i = 0; i < walls.size(); ++i) {
        for (size_t j = i + 1; j < walls.size(); ++j) {
            if (isAdjacent(walls[i], walls[j]) && !walls[i].explored && !walls[j].explored) {
                Eigen::Vector3f normal1(walls[i].normal.normal_x, walls[i].normal.normal_y, walls[i].normal.normal_z);
                Eigen::Vector3f normal2(walls[j].normal.normal_x, walls[j].normal.normal_y, walls[j].normal.normal_z);

                double angleRad = calculateAngle(normal1, normal2);

                walls[i].angle = angleRad;
                walls[j].angle = angleRad;
                walls[i].explored = true;
                walls[j].explored = true;


                wall_seg::WallInfo wall_msg;
                wall_msg.header.stamp = ros::Time::now();
                wall_msg.header.frame_id = "world";
                wall_msg.wall_id = walls[i].id;
                wall_msg.angle = angleRad;
                wall_msg.centroid.x = walls[i].centroid.x();
                wall_msg.centroid.y = walls[i].centroid.y();
                wall_msg.centroid.z = walls[i].centroid.z();
                wall_msg.normal.x = walls[i].normal.normal_x;
                wall_msg.normal.y = walls[i].normal.normal_y;
                wall_msg.normal.z = walls[i].normal.normal_z;
                Eigen::Vector3d corner_position = (walls[i].centroid + walls[j].centroid) / 2.0;
                wall_msg.corner_position.x = corner_position.x();
                wall_msg.corner_position.y = corner_position.y();
                wall_msg.corner_position.z = corner_position.z();

                wall_msg.min.x = walls[i].min.x;
                wall_msg.min.y = walls[i].min.y;
                wall_msg.min.z = walls[i].min.z;
                wall_msg.max.x = walls[i].max.x;
                wall_msg.max.y = walls[i].max.y;
                wall_msg.max.z = walls[i].max.z;



                pub_wall_info.publish(wall_msg);
                break;
            }
        }
    }
}


void curPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    cur_position.header = msg->header;
    cur_position.point = msg->pose.position;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_seg_node");
    ros::NodeHandle nh("~");

    nh.param<double>("wall_seg/distanceThreshold", distanceThreshold, 0.2);
    nh.param<double>("wall_seg/angleThreshold", angleThreshold, 10);
    nh.param<int>("wall_seg/minClusterSize", minClusterSize, 100);
    nh.param<int>("wall_seg/maxClusterSize", maxClusterSize, 10000);
    nh.param<int>("wall_seg/numberOfNeighbours", numberOfNeighbours, 20);
    nh.param<double>("wall_seg/smoothnessThreshold", smoothnessThreshold, 2.0);
    nh.param<double>("wall_seg/radiusSearch", radiusSearch, 0.5);
    nh.param<double>("wall_seg/interestRadius", interestRadius, 5.0);


    pub_clusters = nh.advertise<sensor_msgs::PointCloud2>("colored_clusters_output", 1);
    pub_normals = nh.advertise<visualization_msgs::MarkerArray>("normals", 10);
    pub_wall_info = nh.advertise<wall_seg::WallInfo>("/wall_info", 1);

    ros::Subscriber sub_pcfromfuel = nh.subscribe("/sdf_map/occupancy_all", 10, pointcloudInfoCallback);
    ros::Subscriber sub_cur_position = nh.subscribe("/odom_visualization/pose", 10, curPositionCallback); // Subscribe to drone's current position


    ros::spin();
    return 0;
}

