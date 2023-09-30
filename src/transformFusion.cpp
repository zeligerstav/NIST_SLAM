#include "utility.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer { // extends ParamNode
	public:
		// Fields
		std::mutex mtx; // thread lockout
		double lidarOdomTime = -1;

		// Standard Queues
		deque<nav_msgs::msg::Odometry> imuOdomQueue;

		// Subcriber and Publisher Pointers
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
		rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
		rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

		// Eigen Fields
		Eigen::Isometry3d lidarOdomAffine;
		Eigen::Isometry3d imuOdomAffineFront;
		Eigen::Isometry3d imuOdomAffineBack;

		// TF Fields
		std::shared_ptr<tf2_ros::Buffer> tfBuffer;
		std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
		std::shared_ptr<tf2_ros::TransformListener> tfListener;
		tf2::Stamped<tf2::Transform> lidar2Baselink;

		// Constructor
		TransformFusion(const rclpp::NodeOptions & options) : ParamServer("lio_sam_transformFusion", options) {
			// TF stuff
			tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
			tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

			// Callback Setup
			callbackGroupImuOdometry = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			callbackGroupLaserOdometry = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			auto imuOdomOpt = rclcpp::SubscriptionOptions();
			imuOdomOpt.callback_group = callbackGroupImuOdometry;
			auto laserOdomOpt = rclcpp::SubscriptionOptions();
			laserOdomOpt.callback_group = callbackGroupLaserOdometry;

			// Subscribers
			subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
					"lio_sam/mapping/odometry", qos,
					std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1),
					laserOdomOpt);
			subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(
					odomTopic+"_incremental", qos_imu,
					std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1),
					imuOdomOpt);

			// Publishers
			pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
			pubImuPath = create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);

			// TF Broadcaster
			tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
		}

		// Transform Odom Message to Eigen(?)
		Eigen::Isometry3d odom2affine(nav_msgs::msg::Odometry odom) {
			tf2::Transform t;
			tf2::fromMsg(odom.pose.pose t);
			return tf2::transformToEigen(tf2::toMsg(t));
		}

		// LiDAR Odom Handler
		void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
			std::lock_guard<std::mutex> lock(mtx); // lock this thread

			lidarOdomAffine = odom2affine(*odomMsg);
			lidarOdomTime = stamp2Sec(odomMsg->hedaer.stamp);
			return;
		}

		// IMU Odom Handler
		void imuOdometryHandler(const nav_msgs::msg:Odometry::SharedPtr odomMsg) {
			std::lock_guard<std::mutex> lock(mtx): // lock this thread
			imuOdomQueue.push_back(*odomMsg); // push this IMU odom message into the queue

			if (lidarOdomTime == -1) // we need LiDAR odometry data first
				return;

			while (!imuOdomQueue.empty()) { // pop until we're at the latest IMU odom message that matches the LiDAR odom message
				if (stampe2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
					imuOdomQueue.pop_front();
				else
					break;
			}

			// Transform Eigen representation to tf2
			Eigen::Isometry3d imuOdomAffineFront = odom2affine(imuOdomQueue.front());
			Eigen::Isometry3d imuOdomAffineBack = odom2affine(imuOdomQueue.back());
			Eigen::Isometry3d imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
			Eigen::Isometry3d imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
			auto t = tf2::eigenToTransform(imuOdomAffineLast);
			tf2::Stamped<tf2::Transform> tCur;
			tf2::convert(t, tCur);

			// Build and publish LiDAR odom that matches this IMU odom
			nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
			laserOdometry.pose.pose.position.x = t.transform.translation.x;
			laserOdometry.pose.pose.position.y = t.transform.translation.y;
			laserOdometry.pose.pose.position.z = t.transform.translation.z;
			laserOdometry.pose.pose.orientation = t.transform.rotation;
			pubImuOdometry->publish(laserOdometry);

			// Publish TF
			if(lidarFrame != baselinkFrame)
			{
				try
				{
					tf2::fromMsg(tfBuffer->lookupTransform(
								lidarFrame, baselinkFrame, rclcpp::Time(0)), lidar2Baselink);
				}
				catch (tf2::TransformException ex)
				{
					RCLCPP_ERROR(get_logger(), "%s", ex.what());
				}
				tf2::Stamped<tf2::Transform> tb(
						tCur * lidar2Baselink, tf2_ros::fromMsg(odomMsg->header.stamp), odometryFrame);
				tCur = tb;
			}
			geometry_msgs::msg::TransformStamped ts;
			tf2::convert(tCur, ts);
			ts.child_frame_id = baselinkFrame;
			tfBroadcaster->sendTransform(ts);

			// Publish IMU path
			static nav_msgs::msg::Path imuPath;
			static double last_path_time = -1;
			double imuTime = stamp2Sec(imuOdomQueue.back().header.stamp);
			if (imuTime - last_path_time > 0.1)
			{
				last_path_time = imuTime;
				geometry_msgs::msg::PoseStamped pose_stamped;
				pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
				pose_stamped.header.frame_id = odometryFrame;
				pose_stamped.pose = laserOdometry.pose.pose;
				imuPath.poses.push_back(pose_stamped);
				while(!imuPath.poses.empty() && stamp2Sec(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0)
					imuPath.poses.erase(imuPath.poses.begin());
				if (pubImuPath->get_subscription_count() != 0) // does ROS2 not already handle this?
				{
					imuPath.header.stamp = imuOdomQueue.back().header.stamp;
					imuPath.header.frame_id = odometryFrame;
					pubImuPath->publish(imuPath);
				}
			}
		}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv); // init ROS2

	// ROS2 Node Multithreading TODO: do this manually
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	rclcpp::executors::MultiThreadedExecutor exec; // ROS2 callback multithreading
	exec.add_node(std::make_shared<TransformFusion>(options));

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TransformFusion is alive.");
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
