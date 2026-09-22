/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace common {

    struct Odometry {
        double timestamp;                                      // Unit: s
        Eigen::Vector3d position;                              // IMU origin in internal-world axes, unit: m
        Eigen::Vector3d velocity;                              // IMU-origin velocity in internal-world axes, unit: m/s
        Eigen::Quaterniond orientation;                        // Rotation from IMU axes to internal-world axes
        Eigen::Vector3d angular_velocity;                      // Angular velocity expressed in IMU axes, unit: rad/s
        Eigen::Vector3d gravity;                               // Gravity expressed in internal-world axes
        Eigen::Vector3d translation_lidar_imu_from_lidar_link;// LiDAR origin in IMU axes, unit: m
        Eigen::Matrix3d rotation_lidar_imu_from_lidar_link;    // Rotation from LiDAR axes to IMU axes
    };

    struct ImuMsg {
        double timestamp;                   // Unit: s
        Eigen::Vector3d linear_acceleration;// Unit: g
        Eigen::Vector3d angular_velocity;   // Unit: rad/s
    };

    struct Point {
        double timestamp;        // Unit: s
        Eigen::Vector3f position;// Unit: m
    };

}// namespace common
