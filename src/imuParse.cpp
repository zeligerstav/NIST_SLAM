#include "utility.hpp"

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_acceleration.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_imu_status.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class ImuParse : public ParamServer
{
    public:
        ImuParse(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_imu_preintegration", options) 
        {
            // attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            //     "vehicle_attitude", 10, std::bind(&ImuParse::attitudeCallback, this, std::placeholders::_1));

            acceleration_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAcceleration>(
                "vehicle_acceleration", 10, std::bind(&ImuParse::accelerationCallback, this, std::placeholders::_1));

            // angular_velocity_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAngularVelocity>(
            //     "vehicle_angular_velocity", 10, std::bind(&ImuParse::angularVelocityCallback, this, std::placeholders::_1));

            odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "vehicle_odometry", 10, std::bind(&ImuParse::odometryCallback, this, std::placeholders::_1));

            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_parsed", 10);
        }

        void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
        {
            geometry_msgs::msg::Quaternion orientation;
            orientation.x = msg->q[0];
            orientation.y = msg->q[1];
            orientation.z = msg->q[2];
            orientation.w = msg->q[3];

            imu_msg_.orientation = orientation;
            imu_publisher_->publish(imu_msg_);
        }

        void accelerationCallback(const px4_msgs::msg::VehicleAcceleration::SharedPtr msg)
        {
            geometry_msgs::msg::Vector3 linear_acceleration;
            linear_acceleration.x = msg->xyz[0];
            linear_acceleration.y = msg->xyz[1];
            linear_acceleration.z = msg->xyz[2];

            imu_msg_.linear_acceleration = linear_acceleration;
            imu_publisher_->publish(imu_msg_);
        }

        void angularVelocityCallback(const px4_msgs::msg::VehicleAngularVelocity::SharedPtr msg)
        {
            geometry_msgs::msg::Vector3 angular_velocity;
            angular_velocity.x = msg->xyz[0];
            angular_velocity.y = msg->xyz[1];
            angular_velocity.z = msg->xyz[2];

            imu_msg_.angular_velocity = angular_velocity;
            imu_publisher_->publish(imu_msg_);
        }

        void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
        {
            array<double, 9> orientation_covariance = {msg->orientation_variance[1], 0, 0, 0, msg->orientation_variance[2], 0, 0, 0, msg->orientation_variance[3]};

            imu_msg_.orientation_covariance = orientation_covariance;

            geometry_msgs::msg::Vector3 angular_velocity;
            angular_velocity.x = msg->angular_velocity[0];
            angular_velocity.y = msg->angular_velocity[1];
            angular_velocity.z = msg->angular_velocity[2];

            imu_msg_.angular_velocity = angular_velocity;

            geometry_msgs::msg::Quaternion orientation;
            orientation.x = msg->q[0];
            orientation.y = msg->q[1];
            orientation.z = msg->q[2];
            orientation.w = msg->q[3];

            imu_msg_.orientation = orientation;

            imu_publisher_->publish(imu_msg_);
        }

    private:
        rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
        rclcpp::Subscription<px4_msgs::msg::VehicleAcceleration>::SharedPtr acceleration_subscription_;
        rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_velocity_subscription_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
        rclcpp::Subscription<px4_msgs::msg::VehicleImuStatus>::SharedPtr imu_status_subscription;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

        sensor_msgs::msg::Imu imu_msg_;
};

int main(int argc, char** argv)
{   
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	rclcpp::executors::SingleThreadedExecutor exec;

	auto node = std::make_shared<ImuParse>(options);
	exec.add_node(node);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;31m [DEV] \033[1;32m IMU hijacking is active.\033[0m");

	exec.spin();
	rclcpp::shutdown();
	return 0;
}