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

        float radian_to_degree(float x)
        {
                return x / M_PI * 180.0;
        }

        // parameters
        std::string topic_name;
        float z_max;
        float ratio_invalid_data;

        std::vector<Point3D> points_one_line;
        std::vector<std::vector<Point3D>> points_all_lines;
        int width, height;

        float frame_roll = 0.0, frame_pitch = 0.0;
        float frames_roll =0.0, frames_pitch = 0.0;
        int frame_num = 0;
};

#endif

} // end of namespace
