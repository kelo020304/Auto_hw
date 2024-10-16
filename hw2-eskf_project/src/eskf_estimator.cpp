//
// Created by ubuntu on 24-2-26.
//

#include "eskf_estimator.h"

EskfEstimator::EskfEstimator() {
    main_thread_ = std::thread(&EskfEstimator::MainProcessThread, this);
}

EskfEstimator::~EskfEstimator() {
    shutdown_ = true;
    main_thread_.join();
}

void EskfEstimator::InitState(const double &time) {
    ROS_WARN("init state");
    state_.q = Eigen::Quaterniond(1, 0, 0, 0);
    state_.p = Eigen::Vector3d(0, 0, 0);
    state_.v = Eigen::Vector3d(0, 0, 0);
    state_.ba = Eigen::Vector3d(0, 0, 0);
    state_.bg = Eigen::Vector3d(0, 0, 0);
    state_.P = Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    state_.time = time;
    gravity_ = Eigen::Vector3d(0, 0, -9.805);
    stateInit_ = true;
    Q_ = Eigen::Matrix<double, StateNoiseIndex::NOISE_TOTAL, StateNoiseIndex::NOISE_TOTAL>::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_NOISE, StateNoiseIndex::ACC_NOISE) =
            ACC_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_NOISE, StateNoiseIndex::GYRO_NOISE) =
            GYRO_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_RANDOM_WALK, StateNoiseIndex::ACC_RANDOM_WALK) =
            ACC_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK) =
            GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
    Rm_ = Eigen::Matrix3d::Identity() * WHEEL_ODOMETER_VAR;
    ROS_WARN("init state finish");
}

void EskfEstimator::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    m_buf_.lock();
    imu_buf_.emplace(imu_msg);
    while (imu_buf_.size() > 1000) {
        imu_buf_.pop();
        ROS_WARN("throw imu measurement! %lf", imu_msg->header.stamp.toSec());
    }
    m_buf_.unlock();
}

void EskfEstimator::WheelCallback(const nav_msgs::OdometryConstPtr &wheel_msg) {
    m_buf_.lock();
    odom_buf_.emplace(wheel_msg);
    while (odom_buf_.size() > 1000) {
        odom_buf_.pop();
        ROS_WARN("throw wheel measurement! %lf", wheel_msg->header.stamp.toSec());
    }
    m_buf_.unlock();
}

void EskfEstimator::RosNodeRegistration(ros::NodeHandle &n) {
    sub_imu_ = n.subscribe("/data_generator/imu", 2000, &EskfEstimator::ImuCallback, this,
                           ros::TransportHints().tcpNoDelay());
    sub_wheel_ = n.subscribe("/data_generator/odometry", 2000, &EskfEstimator::WheelCallback, this,
                             ros::TransportHints().tcpNoDelay());
    pub_path_ = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    path_.header.frame_id = "world";
}

void EskfEstimator::MainProcessThread() {
    while (!shutdown_) {
        bool is_update = false;
        sensor_msgs::ImuConstPtr p_imu = nullptr;
        nav_msgs::OdometryConstPtr p_odom = nullptr;

        m_buf_.lock();
        if (!imu_buf_.empty() && !odom_buf_.empty()) {
            p_odom = odom_buf_.front();
            double time_odom = p_odom->header.stamp.toSec();
            if (imu_buf_.back()->header.stamp.toSec() >= time_odom) {
                while (1) {
                    p_imu = imu_buf_.front();
                    double time = p_imu->header.stamp.toSec();
                    Eigen::Vector3d acc(p_imu->linear_acceleration.x,
                                        p_imu->linear_acceleration.y,
                                        p_imu->linear_acceleration.z);
                    Eigen::Vector3d gyro(p_imu->angular_velocity.x,
                                         p_imu->angular_velocity.y,
                                         p_imu->angular_velocity.z);
                    ROS_INFO("deal with IMU. time stamp: %lf", time);
                    PredictByImu(acc, gyro, time);
                    imu_buf_.pop();
                    if (imu_buf_.empty() || imu_buf_.front()->header.stamp.toSec() > time_odom) {
                        break;
                    }
                }
                Eigen::Vector3d wheelSpeed(p_odom->twist.twist.linear.x,
                                           p_odom->twist.twist.linear.y,
                                           p_odom->twist.twist.linear.z);

                double time = p_odom->header.stamp.toSec();
                ROS_INFO("deal with wheel. time stamp: %lf", time);
                UpdateByWheel(time, wheelSpeed);
                is_update = true;
                odom_buf_.pop();
            }
        }
        m_buf_.unlock();

        if(is_update)
            Publish();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void EskfEstimator::PredictByImu(const Eigen::Vector3d &imuAcc, const Eigen::Vector3d &imuGyro, const double &t) {






    if (!stateInit_) {
        InitState(t);
    }

    // YOUR_CODE
    //caculate time delta
    double dt = t - state_.time;
    std::cout<< " dt " << dt << std::endl;
    // mean prediction
    // *** assignment I ***
    const Eigen::Vector3d acc = state_.q * (imuAcc) + gravity_;
    const Eigen::Vector3d omg = (imuGyro) * dt / 2.0;
    Eigen::Quaterniond dq(1.0, omg[0] , omg[1], omg[2]);
    state_.q = (state_.q * dq ).normalized();
    state_.p = state_.p + state_.v * dt + 0.5 * acc * dt * dt;
    state_.v = state_.v + acc * dt;

    // variance propogation
    state_.time = t;

    Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL> F = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);

    // 填充状态转移矩阵 F
    F.block<3, 3>(StateIndex::P, StateIndex::V) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(0,0) = Eigen::Matrix3d::Identity() - SkewSymmetric(imuGyro - state_.bg) * dt;
    // std::cout<< " F1 " << F << std::endl;
    // std::cout<< " imuAcc - state_.ba "  << imuAcc - state_.ba << std::endl;
    F.block<3, 3>(StateIndex::V, 0) = -state_.q.toRotationMatrix() * SkewSymmetric(imuAcc - state_.ba) * dt;
    std::cout<< " F2 " << F << std::endl;
//     F.block<3, 3>(StateIndex::ATTITUDE, StateIndex::GYRO_BIAS) = -Eigen::Matrix3d::Identity() * dt;
}

void EskfEstimator::UpdateByWheel(const double &time, const Eigen::Vector3d &wheelSpeed) {
    // YOUR_CODE

}

Eigen::Matrix3d EskfEstimator::SkewSymmetric(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew;
    skew<< 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return skew;
}

void EskfEstimator::Publish() {
    double time = state_.time;
    Eigen::Vector3d position = state_.p;
    Eigen::Quaterniond q = state_.q;
    Eigen::Vector3d velocity = state_.v;
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "world";
    odometry.header.stamp = ros::Time(time);
    odometry.pose.pose.position.x = position(0);
    odometry.pose.pose.position.y = position(1);
    odometry.pose.pose.position.z = position(2);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();
    odometry.twist.twist.linear.x = velocity(0);
    odometry.twist.twist.linear.y = velocity(1);
    odometry.twist.twist.linear.z = velocity(2);
    odometry.twist.covariance[0] = state_.ba.x();
    odometry.twist.covariance[1] = state_.ba.y();
    odometry.twist.covariance[2] = state_.ba.z();
    odometry.twist.covariance[3] = state_.bg.x();
    odometry.twist.covariance[4] = state_.bg.y();
    odometry.twist.covariance[5] = state_.bg.z();
    pub_odometry_.publish(odometry);

    ROS_INFO("IMU ACC Bias %lf %lf %lf GYRO Bias %lf %lf %lf \n", state_.ba.x(), state_.ba.y(), state_.ba.z(),
             state_.bg.x(), state_.bg.y(), state_.bg.z());

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = ros::Time(time);
    pose_stamped.pose = odometry.pose.pose;
    path_.poses.push_back(pose_stamped);
    pub_path_.publish(path_);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q_tf;
    transform.setOrigin(tf::Vector3(state_.p(0),
                                    state_.p(1),
                                    state_.p(2)));
    q_tf.setW(state_.q.w());
    q_tf.setX(state_.q.x());
    q_tf.setY(state_.q.y());
    q_tf.setZ(state_.q.z());
    transform.setRotation(q_tf);
    br.sendTransform(tf::StampedTransform(transform, pose_stamped.header.stamp, "world", "eskf"));
}