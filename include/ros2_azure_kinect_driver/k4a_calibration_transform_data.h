// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_CALIBRATION_TRANSFORM_DATA_H
#define K4A_CALIBRATION_TRANSFORM_DATA_H

// System headers
//
#include <vector>
#include <memory>

// Library headers
//
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
//#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Project headers
//
#include "ros2_azure_kinect_driver/k4a_ros_device_params.h"

class K4ACalibrationTransformData
{
public:

    K4ACalibrationTransformData(rclcpp::Node* node);

    void initialize(const k4a::device& device, k4a_depth_mode_t depthMode, k4a_color_resolution_t resolution);
    void initialize(const k4a::playback& k4a_playback_handle);
    int getDepthWidth();
    int getDepthHeight();
    int getColorWidth();
    int getColorHeight();
    void getDepthCameraInfo(std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info);
    void getRgbCameraInfo(std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info);
    void print();

    k4a::calibration k4a_calibration_;
    k4a::transformation k4a_transformation_;

    k4a::image point_cloud_image_;
    k4a::image transformed_rgb_image_;
    k4a::image transformed_depth_image_;

    std::string tf_prefix_ = "";
    std::string camera_base_frame_ = "camera_base";
    std::string rgb_camera_frame_ = "rgb_camera_link";
    std::string depth_camera_frame_ = "depth_camera_link";
    std::string imu_frame_ = "imu_link";

private:
    void initialize();

    void printCameraCalibration(k4a_calibration_camera_t& calibration);
    void printExtrinsics(k4a_calibration_extrinsics_t& extrinsics);

    void publishRgbToDepthTf();
    void publishImuToDepthTf();
    void publishDepthToBaseTf();

    static tf2::Quaternion getDepthToBaseRotationCorrection();
    static tf2::Vector3 getDepthToBaseTranslationCorrection();

    rclcpp::Node* node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

#endif
