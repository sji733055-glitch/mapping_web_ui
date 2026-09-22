/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "small_point_lio_node.hpp"
#include "io/pcd_io.h"
#include "lidar_adapter/custom_mid360_driver.h"
#include "lidar_adapter/livox_custom_msg.h"
#include "lidar_adapter/livox_pointcloud2.h"
#include "lidar_adapter/unitree_lidar.h"
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace small_point_lio {

    SmallPointLioNode::SmallPointLioNode(const rclcpp::NodeOptions &options)
        : Node("small_point_lio", options) {
        std::string lidar_topic = declare_parameter<std::string>("lidar_topic");
        std::string imu_topic = declare_parameter<std::string>("imu_topic");
        std::string lidar_type = declare_parameter<std::string>("lidar_type");
        std::string lidar_frame = declare_parameter<std::string>("lidar_frame");
        std::string odom_frame = declare_parameter<std::string>("odom_frame", "odom");
        std::string base_frame = declare_parameter<std::string>("base_frame", "base_link");
        bool publish_odom_tf = declare_parameter<bool>("publish_odom_tf", true);
        bool align_odom_with_gravity = declare_parameter<bool>("align_odom_with_gravity", true);
        bool save_pcd = declare_parameter<bool>("save_pcd");
        small_point_lio = std::make_unique<small_point_lio::SmallPointLio>(*this);
        odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/Odometry", rclcpp::QoS(10).best_effort());
        // 点云用 SensorDataQoS().keep_last(1)：best_effort + 深度 1，让 ROGMap 之类的下游永远消费最新一帧。
        // 深度 10 在高负载时会让旧帧在 rmw 队列里排队，下游拿到的是几十毫秒前的点云，
        // 膨胀/ESDF 也就跟着滞后；点云是可丢的传感数据，宁可丢帧也不要攒延迟。
        // /cloud_registered_full 是稠密去畸变点云（主输出），/cloud_registered 保留同样内容，
        // 供 ROG-Map、odom_localizer 和离线 /map_save 继续使用。
        pointcloud_full_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_full", rclcpp::SensorDataQoS().keep_last(1));
        pointcloud_publisher = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", rclcpp::SensorDataQoS().keep_last(1));
        if (publish_odom_tf) {
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        if (save_pcd) {
            pointcloud_mapping = std::make_unique<util::PointcloudMapping>(0.02);
        }
        map_save_trigger = create_service<std_srvs::srv::Trigger>(
                "map_save",
                [this, save_pcd, lidar_frame](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
                    if (!save_pcd) {
                        res->success = false;
                        res->message = "pcd save is disabled";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "pcd save is disabled");
                        return;
                    }
                    if (!odom_frame_initialized) {
                        res->success = false;
                        res->message = "odom frame is not initialized yet";
                        RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "odom frame is not initialized yet");
                        return;
                    }
                    const std::string transform_path = ROOT_DIR + "/pcd/scan_T_odom_from_internal.txt";
                    {
                        std::ofstream out(transform_path);
                        if (!out) {
                            res->success = false;
                            res->message = "failed to write " + transform_path;
                            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "failed to write %s", transform_path.c_str());
                            return;
                        }
                        const tf2::Matrix3x3 rotation = transform_odom_from_internal_world.getBasis();
                        const tf2::Vector3 translation = transform_odom_from_internal_world.getOrigin();
                        out << std::setprecision(17);
                        out << "# T_odom_from_internal_world 4x4 row-major\n";
                        out << "# p_odom = R * p_internal + t ; same frame as /cloud_registered\n";
                        for (int row = 0; row < 3; ++row) {
                            out << rotation[row][0] << ' ' << rotation[row][1] << ' '
                                << rotation[row][2] << ' ' << translation[row] << '\n';
                        }
                        out << "0 0 0 1\n";
                    }
                    res->success = true;
                    res->message = "saving scan.pcd (LIO-internal world) and " + transform_path;
                    RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "waiting for pcd saving ...");
                    auto pointcloud_to_save = std::make_shared<std::vector<Eigen::Vector3f>>();
                    *pointcloud_to_save = pointcloud_mapping->get_points();
                    std::thread([pointcloud_to_save, lidar_frame]() {
                        io::pcd::write_pcd(ROOT_DIR + "/pcd/scan.pcd", *pointcloud_to_save);
                        RCLCPP_INFO(rclcpp::get_logger("small_point_lio"), "save pcd success");
                    }).detach();
                });
        small_point_lio->set_odometry_callback([this, lidar_frame, odom_frame, base_frame, align_odom_with_gravity, publish_odom_tf](const common::Odometry &odometry) {
            last_odometry = odometry;

            builtin_interfaces::msg::Time time_msg;
            time_msg.sec = std::floor(odometry.timestamp);
            time_msg.nanosec = static_cast<uint32_t>((odometry.timestamp - time_msg.sec) * 1e9);

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = time_msg;
            transform_stamped.header.frame_id = odom_frame;
            transform_stamped.child_frame_id = base_frame;
            geometry_msgs::msg::TransformStamped transform_base_link_from_lidar_link_msg;
            try {
                transform_base_link_from_lidar_link_msg = tf_buffer->lookupTransform(base_frame, lidar_frame, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "Failed to lookup transform from base_link to %s: %s", lidar_frame.c_str(), ex.what());
                return;
            }

            tf2::Transform transform_base_link_from_lidar_link;
            tf2::fromMsg(transform_base_link_from_lidar_link_msg.transform, transform_base_link_from_lidar_link);
            const auto &rotation_lidar_imu_from_lidar_link = odometry.rotation_lidar_imu_from_lidar_link;
            tf2::Transform transform_lidar_imu_from_lidar_link(
                    tf2::Matrix3x3(
                            rotation_lidar_imu_from_lidar_link(0, 0), rotation_lidar_imu_from_lidar_link(0, 1), rotation_lidar_imu_from_lidar_link(0, 2),
                            rotation_lidar_imu_from_lidar_link(1, 0), rotation_lidar_imu_from_lidar_link(1, 1), rotation_lidar_imu_from_lidar_link(1, 2),
                            rotation_lidar_imu_from_lidar_link(2, 0), rotation_lidar_imu_from_lidar_link(2, 1), rotation_lidar_imu_from_lidar_link(2, 2)),
                    tf2::Vector3(
                            odometry.translation_lidar_imu_from_lidar_link.x(),
                            odometry.translation_lidar_imu_from_lidar_link.y(),
                            odometry.translation_lidar_imu_from_lidar_link.z()));
            const tf2::Transform transform_base_link_from_lidar_imu =
                    transform_base_link_from_lidar_link * transform_lidar_imu_from_lidar_link.inverse();

            tf2::Transform transform_internal_world_from_lidar_imu;
            transform_internal_world_from_lidar_imu.setOrigin(
                    tf2::Vector3(odometry.position.x(), odometry.position.y(), odometry.position.z()));
            transform_internal_world_from_lidar_imu.setRotation(
                    tf2::Quaternion(
                            odometry.orientation.x(), odometry.orientation.y(),
                            odometry.orientation.z(), odometry.orientation.w()));

            if (!odom_frame_initialized) {
                if (align_odom_with_gravity) {
                    const double gravity_norm = odometry.gravity.norm();
                    if (!std::isfinite(gravity_norm) || gravity_norm < 1e-6) {
                        RCLCPP_ERROR_THROTTLE(
                                get_logger(), *get_clock(), 5000,
                                "Cannot initialize gravity-aligned odom: invalid gravity vector");
                        return;
                    }

                    const Eigen::Quaterniond rotation_level_from_internal_world_quaternion =
                            Eigen::Quaterniond::FromTwoVectors(
                                    odometry.gravity / gravity_norm,
                                    Eigen::Vector3d(0.0, 0.0, -1.0));
                    const Eigen::Matrix3d rotation_level_from_internal_world_eigen =
                            rotation_level_from_internal_world_quaternion.toRotationMatrix();
                    const tf2::Matrix3x3 rotation_level_from_internal_world(
                            rotation_level_from_internal_world_eigen(0, 0), rotation_level_from_internal_world_eigen(0, 1), rotation_level_from_internal_world_eigen(0, 2),
                            rotation_level_from_internal_world_eigen(1, 0), rotation_level_from_internal_world_eigen(1, 1), rotation_level_from_internal_world_eigen(1, 2),
                            rotation_level_from_internal_world_eigen(2, 0), rotation_level_from_internal_world_eigen(2, 1), rotation_level_from_internal_world_eigen(2, 2));

                    const tf2::Matrix3x3 rotation_level_from_base_link =
                            rotation_level_from_internal_world *
                            transform_internal_world_from_lidar_imu.getBasis() *
                            transform_base_link_from_lidar_imu.inverse().getBasis();
                    const tf2::Vector3 base_forward_axis_in_level =
                            rotation_level_from_base_link * tf2::Vector3(1.0, 0.0, 0.0);
                    double initial_base_yaw = 0.0;
                    if (std::hypot(base_forward_axis_in_level.x(), base_forward_axis_in_level.y()) > 1e-6) {
                        initial_base_yaw = std::atan2(base_forward_axis_in_level.y(), base_forward_axis_in_level.x());
                    }
                    tf2::Quaternion rotation_odom_from_level_quaternion;
                    rotation_odom_from_level_quaternion.setRPY(0.0, 0.0, -initial_base_yaw);
                    const tf2::Matrix3x3 rotation_odom_from_internal_world =
                            tf2::Matrix3x3(rotation_odom_from_level_quaternion) * rotation_level_from_internal_world;

                    const tf2::Transform transform_internal_world_from_base_link_initial =
                            transform_internal_world_from_lidar_imu *
                            transform_base_link_from_lidar_imu.inverse();
                    transform_odom_from_internal_world.setBasis(rotation_odom_from_internal_world);
                    transform_odom_from_internal_world.setOrigin(
                            -(rotation_odom_from_internal_world *
                              transform_internal_world_from_base_link_initial.getOrigin()));
                } else {
                    transform_odom_from_internal_world = transform_base_link_from_lidar_imu;
                }
                odom_frame_initialized = true;
            }

            // Keep odom gravity-aligned while expressing the estimated IMU motion at base_link.
            const tf2::Transform transform_odom_from_base_link =
                    transform_odom_from_internal_world *
                    transform_internal_world_from_lidar_imu *
                    transform_base_link_from_lidar_imu.inverse();
            transform_stamped.transform = tf2::toMsg(transform_odom_from_base_link);

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = time_msg;
            odometry_msg.header.frame_id = odom_frame;
            odometry_msg.child_frame_id = base_frame;
            odometry_msg.pose.pose.position.x = transform_stamped.transform.translation.x;
            odometry_msg.pose.pose.position.y = transform_stamped.transform.translation.y;
            odometry_msg.pose.pose.position.z = transform_stamped.transform.translation.z;
            odometry_msg.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
            odometry_msg.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
            odometry_msg.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
            odometry_msg.pose.pose.orientation.w = transform_stamped.transform.rotation.w;

            const tf2::Vector3 velocity_lidar_imu_origin_in_internal_world(
                    odometry.velocity.x(), odometry.velocity.y(), odometry.velocity.z());
            const tf2::Vector3 velocity_lidar_imu_origin_in_lidar_imu =
                    transform_internal_world_from_lidar_imu.getBasis().transpose() *
                    velocity_lidar_imu_origin_in_internal_world;
            const tf2::Vector3 angular_velocity_in_lidar_imu(
                    odometry.angular_velocity.x(), odometry.angular_velocity.y(), odometry.angular_velocity.z());
            const tf2::Vector3 angular_velocity_in_base_link =
                    transform_base_link_from_lidar_imu.getBasis() * angular_velocity_in_lidar_imu;
            const tf2::Vector3 velocity_base_link_origin_in_base_link =
                    transform_base_link_from_lidar_imu.getBasis() * velocity_lidar_imu_origin_in_lidar_imu +
                    transform_base_link_from_lidar_imu.getOrigin().cross(angular_velocity_in_base_link);

            odometry_msg.twist.twist.linear.x = velocity_base_link_origin_in_base_link.x();
            odometry_msg.twist.twist.linear.y = velocity_base_link_origin_in_base_link.y();
            odometry_msg.twist.twist.linear.z = velocity_base_link_origin_in_base_link.z();
            odometry_msg.twist.twist.angular.x = angular_velocity_in_base_link.x();
            odometry_msg.twist.twist.angular.y = angular_velocity_in_base_link.y();
            odometry_msg.twist.twist.angular.z = angular_velocity_in_base_link.z();

            if (publish_odom_tf) {
                tf_broadcaster->sendTransform(transform_stamped);
            }
            odometry_publisher->publish(odometry_msg);
        });
        small_point_lio->set_pointcloud_callback([this, save_pcd, odom_frame](const std::vector<Eigen::Vector3f> &pointcloud) {
            // 只在真的有人订阅时才组装消息：没有订阅者时序列化这一帧稠密点云纯属浪费。
            // 注意 get_subscription_count() 只统计 QoS 兼容（即 matched）的订阅，
            // 所以 reliable 的订阅端不会点亮这个门限，也就一条都收不到。
            const bool has_full_subscriber = pointcloud_full_publisher->get_subscription_count() > 0;
            const bool has_legacy_subscriber = pointcloud_publisher->get_subscription_count() > 0;
            if (has_full_subscriber || has_legacy_subscriber) {
                builtin_interfaces::msg::Time time_msg;
                time_msg.sec = std::floor(last_odometry.timestamp);
                time_msg.nanosec = static_cast<uint32_t>((last_odometry.timestamp - time_msg.sec) * 1e9);

                if (!odom_frame_initialized) {
                    return;
                }
                const tf2::Matrix3x3 &rotation_odom_from_internal_world_tf2 =
                        transform_odom_from_internal_world.getBasis();
                Eigen::Matrix3f rotation_odom_from_internal_world;
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 3; ++column) {
                        rotation_odom_from_internal_world(row, column) =
                                static_cast<float>(rotation_odom_from_internal_world_tf2[row][column]);
                    }
                }
                const tf2::Vector3 &translation_odom_from_internal_world_tf2 =
                        transform_odom_from_internal_world.getOrigin();
                const Eigen::Vector3f translation_odom_from_internal_world(
                        static_cast<float>(translation_odom_from_internal_world_tf2.x()),
                        static_cast<float>(translation_odom_from_internal_world_tf2.y()),
                        static_cast<float>(translation_odom_from_internal_world_tf2.z()));
                sensor_msgs::msg::PointCloud2 msg;
                msg.header.stamp = time_msg;
                msg.header.frame_id = odom_frame;
                msg.width = pointcloud.size();
                msg.height = 1;
                msg.fields.reserve(4);
                sensor_msgs::msg::PointField field;
                field.name = "x";
                field.offset = 0;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "y";
                field.offset = 4;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "z";
                field.offset = 8;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                field.name = "intensity";
                field.offset = 12;
                field.datatype = sensor_msgs::msg::PointField::FLOAT32;
                field.count = 1;
                msg.fields.push_back(field);
                msg.is_bigendian = false;
                msg.point_step = 16;
                msg.row_step = msg.width * msg.point_step;
                msg.data.resize(msg.row_step * msg.height);
                Eigen::Vector3f transformed_point;
                auto pointer = reinterpret_cast<float *>(msg.data.data());
                for (const auto &point: pointcloud) {
                    transformed_point =
                            rotation_odom_from_internal_world * point +
                            translation_odom_from_internal_world;
                    *pointer = transformed_point.x();
                    ++pointer;
                    *pointer = transformed_point.y();
                    ++pointer;
                    *pointer = transformed_point.z();
                    ++pointer;
                    *pointer = 0;
                    ++pointer;
                }
                msg.is_dense = false;
                // 同一份数据发两个话题：组装只做一次，publish 各自拷进中间件。
                if (has_full_subscriber) {
                    pointcloud_full_publisher->publish(msg);
                }
                if (has_legacy_subscriber) {
                    pointcloud_publisher->publish(msg);
                }
            }
            if (save_pcd) {
                for (const auto &point: pointcloud) {
                    pointcloud_mapping->add_point(point);
                }
            }
        });
        if (lidar_type == "livox_custom_msg") {
#ifdef HAVE_LIVOX_DRIVER
            lidar_adapter = std::make_unique<LivoxCustomMsgAdapter>();
#else
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "livox_custom_msg requested but not available!");
            rclcpp::shutdown();
            return;
#endif
        } else if (lidar_type == "livox_pointcloud2") {
            lidar_adapter = std::make_unique<LivoxPointCloud2Adapter>();
        } else if (lidar_type == "custom_mid360_driver") {
            lidar_adapter = std::make_unique<CustomMid360DriverAdapter>();
        } else if (lidar_type == "unilidar") {
            lidar_adapter = std::make_unique<UnilidarAdapter>();
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("small_point_lio"), "unknwon lidar type");
            rclcpp::shutdown();
            return;
        }
        lidar_adapter->setup_subscription(this, lidar_topic, [this](const std::vector<common::Point> &pointcloud) {
            small_point_lio->on_point_cloud_callback(pointcloud);
            small_point_lio->handle_once();
        });
        imu_subsciber = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Imu &msg) {
                    common::ImuMsg imu_msg;
                    imu_msg.angular_velocity = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                    imu_msg.linear_acceleration = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                    imu_msg.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
                    small_point_lio->on_imu_callback(imu_msg);
                    small_point_lio->handle_once();
                });
    }

}// namespace small_point_lio

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(small_point_lio::SmallPointLioNode)
