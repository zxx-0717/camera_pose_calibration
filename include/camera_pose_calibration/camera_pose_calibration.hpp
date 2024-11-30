#ifndef CAMERA_POSE_CALIBRATION__CAMERA_POSE_CALIBRATION_HPP__
#define CAMERA_POSE_CALIBRATION__CAMERA_POSE_CALIBRATION_HPP__

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace camera_pose_calibration{

struct Point3D
{
        float x;
        float y;
        float z;
};

class CameraPoseCalibration : public rclcpp::Node
{
public:
        CameraPoseCalibration(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        ~CameraPoseCalibration();

        // sub for /camera2/depth/points
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera2_points_sub;
        void camera2_points_sub_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void init_params();

        // parameters
        std::string topic_name;
        float z_max;

        std::vector<Point3D> points;
        int width, height;
};

#endif

} // end of namespace
