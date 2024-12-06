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
        RCLCPP_INFO(get_logger(), "distance_max: %f", this->distance_max);
        RCLCPP_INFO(get_logger(), "ration_invalid_data: %f", this->ratio_invalid_data);
        RCLCPP_INFO(get_logger(), "target_frame: %s", this->target_frame.c_str());

        tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);


        camera2_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        this->topic_name, rclcpp::SensorDataQoS(), std::bind(&CameraPoseCalibration::camera2_points_sub_callback, this, _1));
        

}       

CameraPoseCalibration::~CameraPoseCalibration()
{

}

void CameraPoseCalibration::init_params()
{
        this->declare_parameter<float>("distance_max", 0.5);
        this->declare_parameter<std::string>("topic_name", "/camera3/depth/points");
        this->declare_parameter<float>("ratio_invalid_data", 0.4);
        this->declare_parameter<std::string>("target_frame", "base_link");

	this->distance_max = this->get_parameter_or<float>("distance_max", 0.5);
	this->topic_name = this->get_parameter_or<std::string>("topic_name", "/camera3/depth/points");
	this->ratio_invalid_data = this->get_parameter_or<float>("ratio_invalid_data", 0.4);
        this->target_frame = this->get_parameter_or<std::string>("target_frame", "base_link");

}

void CameraPoseCalibration::camera2_points_sub_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
        float sin_theta = 0.08583788;
        float cos_theta = 0.99630912;
        RCLCPP_INFO(get_logger(), "--------------------------- frame %d ---------------------------", frame_num++);

        // try 
        // {
        //         auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        //         tf2_->transform(*msg, *cloud, target_frame, tf2::durationFromSec(0.01));
        //         msg = cloud;
        //         } catch (tf2::TransformException & ex) {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
        //         return;
        // }

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
                int count_invlid = 0;
                for (int col = 85; col < width - 90; col++)
                {
                        index = (row * width + col) * msg->point_step;
                        
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

                        if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z) && (z < this->distance_max))
                        {
                                point.x = x;
                                point.y = y;
                                point.z = z;
                                point.row = row;
                                point.col = col;
                                this->points_one_line.push_back(point);
                                if (z_min > z)
                                {
                                        z_min = z;
                                        // RCLCPP_INFO(get_logger(), "updata z_min: %f", z_min);
                                }
                        }
                        else
                        {
                                count_invlid++;
                                if (count_invlid >= (int)((msg->width - 85 - 90)  * this->ratio_invalid_data))
                                {
                                        break;
                                }
                                else
                                {
                                        continue;
                                }
                        }
                }
                if (this->points_one_line.size() > (int)(msg->width * (1.0 - this->ratio_invalid_data)))
                {
                        this->points_all_lines.push_back(this->points_one_line);
                }
        }

        size_t lines = this->points_all_lines.size();

        float z_mean_min = 5.0;
        float z_mean_max = 0.0;

        float y_mean_min = 5.0;
        float y_mean_max = -5.0;

        RCLCPP_INFO(get_logger(), "lines's number: %zd", lines);
        float pitch = 0.0, roll = 0.0;

        int sign_pitch = 0;

        for (size_t line = 0; line < lines; line++)
        {
                auto points_line = this->points_all_lines[line];
                float z_sum = 0.0;
                float y_sum = 0.0;

                float x_min = 0.2;
                float x_max = -0.2;
                float y_min = 2.0;
                float y_max = -2.0;

                RCLCPP_DEBUG(get_logger(), "line %zd points.size: %zd", line, points_line.size());
                
                int sign_roll = 0;
                
                for (size_t point_index = 0; point_index < points_line.size(); point_index++)
                {
                        auto point = points_line[point_index];
                        float y_, z_;
                        y_ = point.y * cos_theta - z_ * sin_theta;
                        z_ = point.y * sin_theta + z * cos_theta;
                        RCLCPP_DEBUG(get_logger(), "row: %d, col: %d => x: %f, y: %f, z: %f",point.row, point.col, point.x, y_, z_);

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
                                        sign_pitch--;
                                }
                                if (y_mean_min > y_mean)
                                {
                                        y_mean_min = y_mean;
                                        sign_pitch++;
                                }
                        }

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
                                sign_roll--;
                        }
                        if (y_max < point.y)
                        {
                                y_max = point.y;
                                sign_roll++;
                        }
                }

                sign_roll = sign_roll > 0 ? 1 : -1;
                sign_pitch = sign_pitch > 0 ? 1 : -1;

                float roll_cur = std::atan2(y_max - y_min, x_max - x_min) * sign_roll;
                roll += roll_cur;

                RCLCPP_DEBUG(get_logger(), "y_max: %f, y_min: %f, x_max: %f, x_min: %f", y_max, y_min, x_max, x_min);
                RCLCPP_DEBUG(get_logger(), "line %zd roll: %f(%f degree)", line, roll_cur, radian_to_degree(roll_cur));
                frame_roll += roll_cur;

                if (line == lines - 1)
                {
                        RCLCPP_DEBUG(get_logger(), "y_mean_max: %f, y_mean_min: %f, z_mean_max: %f, z_min_min: %f",
                                y_mean_max, y_mean_min, z_mean_max, z_mean_min);
                        pitch = std::atan2(y_mean_max - y_mean_min, z_mean_max - z_mean_min) * sign_pitch;
                        frame_pitch = pitch;
                        frame_roll /= lines;
                }
        }
        

        frames_pitch = frames_pitch * (frame_num - 1) + frame_pitch;
        frames_roll  = frames_roll *(frame_num - 1) + frame_roll;

        frames_pitch /= frame_num;
        frames_roll  /= frame_num;

        RCLCPP_INFO(get_logger(), "current pitch: %f(%f degree)",  frame_pitch, radian_to_degree(frame_pitch));
        RCLCPP_INFO(get_logger(), "average pitch: %f(%f degree)",  frames_pitch, radian_to_degree(frames_pitch));

        RCLCPP_INFO(get_logger(), "current roll : %f(%f degree)",  frame_roll, radian_to_degree(frame_roll));
        RCLCPP_INFO(get_logger(), "average roll : %f(%f degree)",  frames_roll, radian_to_degree(frames_roll));

}

} // end of namespace

RCLCPP_COMPONENTS_REGISTER_NODE(camera_pose_calibration::CameraPoseCalibration)
