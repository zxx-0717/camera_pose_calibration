#include "camera_pose_calibration/camera_pose_calibration.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1, std::placeholders::_2;

namespace camera_pose_calibration{

CameraPoseCalibration::CameraPoseCalibration(const rclcpp::NodeOptions & options):
        rclcpp::Node("camera_pose_calibration", options)
{
        RCLCPP_INFO(get_logger(), "camera_pose_calibration construction.");
        RCLCPP_INFO(get_logger(), "get parameters");
        init_params();
        RCLCPP_INFO(get_logger(), "topic_name: %s", this->topic_name.c_str());
        RCLCPP_INFO(get_logger(), "z_max: %f", this->z_max);


        camera2_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        this->topic_name, rclcpp::SensorDataQoS(), std::bind(&CameraPoseCalibration::camera2_points_sub_callback, this, _1));
        

}       

CameraPoseCalibration::~CameraPoseCalibration()
{

}

void CameraPoseCalibration::init_params()
{
        this->declare_parameter<float>("z_max", 0.5);
        this->declare_parameter<std::string>("topic_name", "/camera3/depth/points");

	this->z_max = this->get_parameter_or<float>("z_max", 0.5);
	this->topic_name = this->get_parameter_or<std::string>("topic_name", "/camera3/depth/points");

}

void CameraPoseCalibration::camera2_points_sub_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
        RCLCPP_INFO(get_logger(), "-----------------");
        this->width = msg->width;
        this->height = msg->height;

        std::vector<Point3D>().swap(this->points);

        Point3D point;
        int index = 0;
        float x, y, z;
        uint8_t uc_x[4], uc_y[4], uc_z[4];

        float z_min = 10.0;

        for (int row = 0; row < height; row++)
        {
                for (int col = 0; col < width; col++)
                {
                        index = (row * width + height) * 16; // msg->point_step = 16;
                        uc_z[0] = msg->data[index + 8];
                        uc_z[1] = msg->data[index + 9];
                        uc_z[2] = msg->data[index + 10];
                        uc_z[3] = msg->data[index + 11];
                        memcpy(&z, uc_z, 4);

                        if (!std::isnan(z) && (z < this->z_max))
                        {
                                uc_x[0] = msg->data[index + 0];
                                uc_x[1] = msg->data[index + 1];
                                uc_x[2] = msg->data[index + 2];
                                uc_x[3] = msg->data[index + 3];
                                memcpy(&x, uc_x, 4);

                                uc_y[0] = msg->data[index + 4];
                                uc_y[1] = msg->data[index + 5];
                                uc_y[2] = msg->data[index + 6];
                                uc_y[3] = msg->data[index + 7];
                                memcpy(&y, uc_y, 4);

                                point.x = x;
                                point.y = y;
                                point.z = z;
                                this->points.push_back(point);
                                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,"row: %d, col: %d => x: %f, y: %f, z: %f",row, col, x, y, z);
                                if (z_min > z)
                                {
                                        z_min = z;
                                        RCLCPP_INFO(get_logger(), "updata z_min: %f", z_min);
                                }
                        }
                        else
                        {
                                continue;
                        }
                }
        }
        RCLCPP_INFO(get_logger(), "size: %zd", this->points.size());
}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(camera_pose_calibration::CameraPoseCalibration)
