#include "utility.hpp"

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuParse : public ParamServer
{
    public: 
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;

        ImuParse(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_imu_preintegration", options) 
        {
            subImu = create_subscription<sensor_msgs::msg::Imu>(
                        "/imu_raw", 10,
                        std::bind(&ImuParse::imu_in, this, std::placeholders::_1));
            
            pubImu = create_publisher<sensor_msgs::msg::Imu>("imu_parsed", 10);
        }

        void imu_in(const sensor_msgs::msg::Imu::SharedPtr imu_raw)
        {
            sensor_msgs::msg::Imu imu_in = *imu_raw;
            auto imu_out = sensor_msgs::msg::Imu();

            imu_out.angular_velocity = imu_in.angular_velocity;
            imu_out.linear_acceleration = imu_in.linear_acceleration;
            imu_out.header = imu_in.header;
            imu_out.orientation.x = 0.0;
            imu_out.orientation.y = 0.0;
            imu_out.orientation.z = 0.0;
            imu_out.orientation.w = 0.0;
            pubImu->publish(imu_out);

            return;
        }
};

int main(int argc, char** argv)
{   
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	rclcpp::executors::SingleThreadedExecutor exec;

	auto node = std::make_shared<ImuParse>(options);
	exec.add_node(node);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[DEV] IMU PARSER IS ACTIVE");

	exec.spin();
	rclcpp::shutdown();
	return 0;
}