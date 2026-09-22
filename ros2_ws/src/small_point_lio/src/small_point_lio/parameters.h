/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num;
        float min_distance_squared;
        float max_distance_squared;
        // 盲区球心，雷达坐标系，单位 m。min_distance 这个盲区球以它为球心，
        // 而不是以雷达原点为球心。雷达装在车体一角时，以雷达为心的球会把车身另一侧
        // 的近场地面一起裁掉；把球心挪到车体中心，同样的半径就只裁车身。
        // max_distance 仍以雷达原点为心——那是量程，不是盲区。
        Eigen::Vector3f blind_center;
        bool space_downsample;
        float space_downsample_leaf_size;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        double satu_acc;
        double satu_gyro;
        double acc_norm;

        double map_resolution;
        size_t init_map_size;

        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double laser_point_cov;
        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double plane_threshold;
        double match_sqaured;

        bool publish_odometry_without_downsample = false;

        void read_parameters(rclcpp::Node &node);
    };

}// namespace small_point_lio
