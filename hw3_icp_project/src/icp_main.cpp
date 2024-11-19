//
// Created by ubuntu on 24-10-12.
//

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/registration/icp.h>

#include <iostream>
#include <iomanip> // std::setw

std::string root_path = "/home/ubuntu/test_ws/src/icp/data/"; // set to your absolute path

struct PointFactor
{
    PointFactor(pcl::PointXYZ &p0, pcl::PointXYZ &p1) : p0(p0), p1(p1) {}
    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Quaternion<T> q_{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> point0{T(p0.x), T(p0.y), T(p0.z)};
        Eigen::Matrix<T, 3, 1> point1{T(p1.x), T(p1.y), T(p1.z)};
        Eigen::Matrix<T, 3, 1> r = point0 - (q_ * point1 + t_);
        residual[0] = r[0];
        residual[1] = r[1];
        residual[2] = r[2];
        return true;
    }
    pcl::PointXYZ p0, p1;
};


pcl::PointCloud<pcl::PointXYZ> IcpByCeres(const pcl::PointCloud<pcl::PointXYZ>& pc0,
                const pcl::PointCloud<pcl::PointXYZ>& pc1) {

    // implement by yourself
    // 1. build kd-tree of pc0: using pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // 2. find the closet point pairs (pc0 --- pc1 )
    // 3. construct ceres problem and solve, get rotation and position
    // 4. update pc1 --> pc1'    repeat step 2, until converge

    pcl::PointCloud<pcl::PointXYZ> pc1_trans;
    return pc1_trans;
}

pcl::PointCloud<pcl::PointXYZ> IcpByPcl(const pcl::PointCloud<pcl::PointXYZ>& pc0,
              const pcl::PointCloud<pcl::PointXYZ>& pc1) {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pc1.makeShared());
    icp.setInputTarget(pc0.makeShared());
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "ICP has " << (icp.hasConverged()?"converged":"not converged") << ", score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    Eigen::Matrix4f transformation;
    transformation = icp.getFinalTransformation();
    Eigen::Matrix3f r_optimal = transformation.block<3, 3>(0, 0);
    Eigen::Vector3f t_optimal = transformation.block<3, 1>(0, 3);
    Eigen::Quaternionf q_optimal(r_optimal);
    std::cout << "q: " << q_optimal.w() << " " << q_optimal.vec().transpose() << std::endl;
    std::cout << "t: " << t_optimal.transpose() << std::endl;

    pcl::PointCloud<pcl::PointXYZ> pc1_trans;

    for (int i = 0; i < pc1.size(); i++) {
        pcl::PointXYZ p = pc1[i];
        Eigen::Vector3f p_trans = r_optimal * Eigen::Vector3f(p.x, p.y, p.z) + t_optimal;
        pc1_trans.emplace_back(p_trans.x(), p_trans.y(), p_trans.z());
    }
    return pc1_trans;
}

pcl::PointCloud<pcl::PointXYZ> IcpBySVD(const pcl::PointCloud<pcl::PointXYZ>& pc0,
                                         const pcl::PointCloud<pcl::PointXYZ>& pc1) {

    // implement by yourself
    // implement by yourself

    // 1. build kd-tree of pc0: using pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pc0.makeShared());
    pcl::PointCloud<pcl::PointXYZ> updated_pc1;
    pcl::PointCloud<pcl::PointXYZ> pc1_trans = pc1;

    double tolerance = 1e-4;
    int max_iterations = 50;
    double prev_error = std::numeric_limits<double>::max();
    int iteration = 0;


    std::vector<int> point_search_idx(1);
    std::vector<float> point_search_sq_dist(1);

    Eigen::MatrixXf src(3, pc1.size());  //存储输入点云的所有点
    Eigen::MatrixXf tgt(3, pc1.size()); //存储对应点云的所有点

    size_t num_points = pc1.size();
    size_t progress_bar_width = 50; // 进度条的宽度
    // 2. find the closet point pairs (pc0 --- pc1 )
    float distance_threshold = 1.0;
    while (iteration < max_iterations) {
        std::vector<int> point_search_idx(1);
        std::vector<float> point_search_sq_dist(1);

        Eigen::MatrixXf src(3, pc1_trans.size());  // 存储输入点云的所有点
        Eigen::MatrixXf tgt(3, pc1_trans.size());  // 存储对应的目标点

        // 2. 找到最近点对 (pc0 --- pc1_trans)
        size_t valid_pairs = 0;
        for (size_t i = 0; i < pc1_trans.size(); i++) {
            pcl::PointXYZ search_point = pc1_trans.points[i];
            if (kdtree.nearestKSearch(search_point, 1, point_search_idx, point_search_sq_dist) > 0) {
                if (point_search_sq_dist[0] < 1.0) { // 使用距离阈值进行过滤
                    pcl::PointXYZ nearest_point = pc0.points[point_search_idx[0]];
                    src(0, valid_pairs) = search_point.x;
                    src(1, valid_pairs) = search_point.y;
                    src(2, valid_pairs) = search_point.z;
                    tgt(0, valid_pairs) = nearest_point.x;
                    tgt(1, valid_pairs) = nearest_point.y;
                    tgt(2, valid_pairs) = nearest_point.z;
                    valid_pairs++;
                }
            }
        }

    if (valid_pairs < 3) {
        std::cerr << "Insufficient valid point pairs for SVD" << std::endl;
        break;
    }

    src.conservativeResize(3, valid_pairs);
    tgt.conservativeResize(3, valid_pairs);
    // 3. construct SVD problem and solve, get rotation and position
    Eigen::Vector3f src_mean = src.rowwise().mean(); // 输入点云质心
    Eigen::Vector3f tgt_mean = tgt.rowwise().mean(); // 目标点云的质心
    Eigen::MatrixXf src_demean = src.colwise() - src_mean;
    Eigen::MatrixXf tgt_demean = tgt.colwise() - tgt_mean;
    Eigen::Matrix3f H = src_demean * tgt_demean.transpose();

    //SVD 分解
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    //compute rotation matrix R
    Eigen::Matrix3f R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }
    // calculate transition matrix t
    Eigen::Vector3f t = tgt_mean - R * src_mean;
    // 4. update pc1 --> pc1'    repeat step 2, until converge
    // pcl::PointCloud<pcl::PointXYZ> pc1_trans;
    updated_pc1.clear();
    for (size_t i = 0; i < pc1_trans.size(); ++i) {
        Eigen::Vector3f p(pc1_trans.points[i].x, pc1_trans.points[i].y, pc1_trans.points[i].z);
        Eigen::Vector3f p_transformed = R * p + t;

        pcl::PointXYZ p_new;
        p_new.x = p_transformed.x();
        p_new.y = p_transformed.y();
        p_new.z = p_transformed.z();
        updated_pc1.push_back(p_new);
    }

    // 计算误差（均方误差）
    double error = 0.0;
        for (size_t i = 0; i < valid_pairs; i++) {
            Eigen::Vector3f p_transformed(updated_pc1.points[i].x, updated_pc1.points[i].y, updated_pc1.points[i].z);
            Eigen::Vector3f p_target(tgt(0, i), tgt(1, i), tgt(2, i));
            error += (p_transformed - p_target).squaredNorm();
        }
        error /= valid_pairs;

        if (std::abs(prev_error - error) < tolerance) {
            std::cout << "Converged at iteration " << iteration << ", error: " << error << std::endl;
            break;
        }

        prev_error = error;
        pc1_trans = updated_pc1;
        iteration++;
    }

    return pc1_trans;
}


std::vector<float> ReadLidarData(const std::string& lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}


void LoadLidarPoint(std::vector<pcl::PointCloud<pcl::PointXYZ>>& laser_cloud_vec){
    std::string dataset_folder, sequence_number, output_bag_file;
    std::string lidar_data_path0 = root_path + "0000000000.bin";
    std::string lidar_data_path1 = root_path + "0000000002.bin";
    std::vector<std::string> lidar_data_paths{lidar_data_path0, lidar_data_path1};
    laser_cloud_vec.clear();

    for (int i = 0; i < lidar_data_paths.size(); ++i)
    {
        std::vector<float> lidar_data = ReadLidarData(lidar_data_paths[i]);
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZ>laser_cloud;

        for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i + 1], lidar_data[i + 2]);
            lidar_intensities.push_back(lidar_data[i + 3]);

            pcl::PointXYZ point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            laser_cloud.push_back(point);
        }
        laser_cloud_vec.emplace_back(laser_cloud);
    }

}

void GeneratePoint(std::vector<pcl::PointCloud<pcl::PointXYZ>>& laser_cloud_vec){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in->width = 50;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    *cloud_target = *cloud_in;

    Eigen::Quaternionf q(1.0, 0.1, 0.1, 0.1);
    Eigen::Matrix3f rotation = q.normalized().toRotationMatrix();
    Eigen::Vector3f t (0.3, 0.4, 0.5);
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        Eigen::Vector3f v(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
        Eigen::Vector3f v_out = rotation * v + t;
        cloud_target->points[i].x = v_out.x();
        cloud_target->points[i].y = v_out.y();
        cloud_target->points[i].z = v_out.z();
    }
    laser_cloud_vec.clear();
    laser_cloud_vec.emplace_back(*cloud_target);
    laser_cloud_vec.emplace_back(*cloud_in);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_node");
    ros::NodeHandle n("~");
    ros::Publisher pub_target_cloud = n.advertise<sensor_msgs::PointCloud2>("/target_cloud", 2);
    ros::Publisher pub_input_cloud = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 2);
    ros::Publisher pub_icp_pcl_result = n.advertise<sensor_msgs::PointCloud2>("/icp_pcl_result", 2);
    ros::Publisher pub_icp_ceres_result = n.advertise<sensor_msgs::PointCloud2>("/icp_ceres_result", 2);
    ros::Publisher pub_icp_svd_result = n.advertise<sensor_msgs::PointCloud2>("/icp_svd_result", 2);

    std::vector<pcl::PointCloud<pcl::PointXYZ>> laser_cloud_vec;

    // 1. Assignment I : simulation data
    GeneratePoint(laser_cloud_vec);

    // 2. Assignment II : real laser scan   #change to your own $root_path$
    // LoadLidarPoint(laser_cloud_vec);

    pcl::PointCloud<pcl::PointXYZ> target_cloud = laser_cloud_vec[0];
    pcl::PointCloud<pcl::PointXYZ> input_cloud = laser_cloud_vec[1];

    pcl::PointCloud<pcl::PointXYZ> output_pcl, output_ceres, output_svd;
    output_pcl = IcpByPcl(target_cloud, laser_cloud_vec[1]);
    output_ceres = IcpByCeres(target_cloud, laser_cloud_vec[1]);
    output_svd = IcpBySVD(target_cloud, laser_cloud_vec[1]);

    sensor_msgs::PointCloud2 laser_cloud_msg0;
    pcl::toROSMsg(target_cloud, laser_cloud_msg0);
    laser_cloud_msg0.header.stamp = ros::Time::now();
    laser_cloud_msg0.header.frame_id = "world";

    sensor_msgs::PointCloud2 laser_cloud_msg1;
    pcl::toROSMsg(input_cloud, laser_cloud_msg1);
    laser_cloud_msg1.header.stamp = ros::Time::now();
    laser_cloud_msg1.header.frame_id = "world";

    sensor_msgs::PointCloud2 laser_cloud_msg2;
    pcl::toROSMsg(output_pcl, laser_cloud_msg2);
    laser_cloud_msg2.header.stamp = ros::Time::now();
    laser_cloud_msg2.header.frame_id = "world";

    sensor_msgs::PointCloud2 laser_cloud_msg3;
    pcl::toROSMsg(output_ceres, laser_cloud_msg3);
    laser_cloud_msg3.header.stamp = ros::Time::now();
    laser_cloud_msg3.header.frame_id = "world";

    sensor_msgs::PointCloud2 laser_cloud_msg4;
    pcl::toROSMsg(output_svd, laser_cloud_msg4);
    laser_cloud_msg4.header.stamp = ros::Time::now();
    laser_cloud_msg4.header.frame_id = "world";

    ros::Rate r(10.0);

    while (ros::ok()){
        pub_target_cloud.publish(laser_cloud_msg0);
        pub_input_cloud.publish(laser_cloud_msg1);
        pub_icp_pcl_result.publish(laser_cloud_msg2);
        pub_icp_ceres_result.publish(laser_cloud_msg3);
        pub_icp_svd_result.publish(laser_cloud_msg4);
        r.sleep();
    }
}