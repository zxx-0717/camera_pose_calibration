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

        Point3D point;
        int index = 0;
        float x, y, z;
        uint8_t uc_x[4], uc_y[4], uc_z[4];

        float z_min = 10.0;
        std::vector<std::vector<Point3D>>().swap(this->points_all_lines);

        for (int row = 0; row < height; row++)
        {
                std::vector<Point3D>().swap(this->points_one_line);
                for (int col = 0; col < width; col++)
                {
                        index = (row * width + height) * 16; // msg->point_step = 16;
                        
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

                        uc_z[0] = msg->data[index + 8];
                        uc_z[1] = msg->data[index + 9];
                        uc_z[2] = msg->data[index + 10];
                        uc_z[3] = msg->data[index + 11];
                        memcpy(&z, uc_z, 4);

                        if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z) && (z < this->z_max))
                        {
                                point.x = x;
                                point.y = y;
                                point.z = z;
                                this->points_one_line.push_back(point);
                                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,"row: %d, col: %d => x: %f, y: %f, z: %f",row, col, x, y, z);
                                if (z_min > z)
                                {
                                        z_min = z;
                                        RCLCPP_DEBUG(get_logger(), "updata z_min: %f", z_min);
                                }
                        }
                        else
                        {
                                continue;
                        }
                }
                if (this->points_one_line.size() > 0)
                {
                        this->points_all_lines.push_back(this->points_one_line);
                }
        }

        size_t lines = this->points_all_lines.size();

        float z_mean_min = 5.0;
        float z_mean_max = 0.0;

        float y_mean_min = 5.0;
        float y_mean_max = -5.0;

        RCLCPP_INFO(get_logger(), "lines.size: %zd", lines);
        for (size_t line = 0; line < lines; line++)
        {
                auto points_line = this->points_all_lines[line];
                float z_sum = 0.0;
                float y_sum = 0.0;

                float x_min = 0.2;
                float x_max = -0.2;
                float y_min = 2.0;
                float y_max = -2.0;

                RCLCPP_INFO(get_logger(), "line %zd points.size: %zd", line, points_line.size());
                for (size_t point_index = 0; point_index < points_line.size(); point_index++)
                {
                        auto point = points_line[point_index];

                        z_sum += point.z;
                        y_sum += point.y;
                        if (point_index == points_line.size() - 1)
                        {
                                float z_mean = z_sum / (float)points_line.size();
                                if (z_mean_max < z_mean)
                                {
                                        z_mean_max = z_mean;
                                }
                                if (z_mean_min > z_mean)
                                {
                                        z_mean_min = z_mean;
                                }

                                float y_mean = y_sum / (float)points_line.size();
                                if (y_mean_max < y_mean)
                                {
                                        y_mean_max = y_mean;
                                }
                                if (y_mean_min > y_mean)
                                {
                                        y_mean_min = y_mean;
                                }
                        }

                        RCLCPP_INFO(get_logger(), "x: %f", point.x);
                        if (x_min > point.x)
                        {
                                x_min = point.x;
                        }
                        if (x_max < point.x)
                        {
                                x_max = point.x;
                        }
                        if (y_min > point.y)
                        {
                                y_min = point.y;
                        }
                        if (y_max < point.y)
                        {
                                y_max = point.y;
                        }
                }
                RCLCPP_INFO(get_logger(), "y_max: %f, y_min: %f, x_max: %f, x_min: %f", y_max, y_min, x_max, x_min);
                RCLCPP_INFO(get_logger(), "line %zd roll: %f", line, std::acos((y_max - y_min)/(x_max - x_min)));

                if (line == this->points_all_lines.size() - 1)
                {
                        RCLCPP_INFO(get_logger(), "y_mean_max: %f, y_mean_min: %f, z_mean_max: %f, z_min_min: %f",
                                 y_mean_max, y_mean_min, z_mean_max, z_mean_min);
                        RCLCPP_INFO(get_logger(), "pitch: %f", std::acos( (y_mean_max - y_mean_min) / (z_mean_max - z_mean_min) ));
                }
                
        }
}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(camera_pose_calibration::CameraPoseCalibration)
