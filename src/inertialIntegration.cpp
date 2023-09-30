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

class InertialIntegration : public ParamNode { // extends ParamServer
	public:
		// Fields
		std::mutex mtx; // thread lockout
		const double delta_t = 0; // TODO: what is this really for? It's a constant zero...
		double lastImuT_imu = -1;
		double lastImuT_opt = -1;
		int key = 1; // tracks number of optimization runs
		bool systemInititalized = false;
		bool doneFirstOpt = false;

		// Subscriber and Publisher pointers
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
		rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
		rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;

		// Standard Queues
		std::deque<sensor_msgs::msg::Imu> imuQueOpt;
		std::deque<sensor_msgs::msg::Imu> imuQueImu;

		// gtsam Fields
		gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
		gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
		gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
		gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
		gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
		gtsam::Vector noiseModelBetweenBias;
		gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
		gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
		gtsam::Pose3 prevPose_;
		gtsam::Vector3 prevVel_;
		gtsam::NavState prevState_;
		gtsam::imuBias::ConstantBias prevBias_;
		gtsam::NavState prevStateOdom;
		gtsam::imuBias::ConstantBias prevBiasOdom;
		gtsam::ISAM2 optimizer;
		gtsam::NonlinearFactorGraph graphFactors;
		gtsam::Values graphValues;

		// gtsam Init
		gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
		gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

		// Constructor
		InertialIntegration(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_inertialIntegration", options) {
			// Callback Groups
			callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        	callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			auto imuOpt = rclcpp::SubscriptionOptions();
			imuOpt.callback_group = callbackGroupImu;
			auto odomOpt = rclcpp::SubscriptionOptions();
			odomOpt.callback_group = callbackGroupOdom;

			// Create Subscribers
			subImu = create_subscription<sensor_msgs::msg::Imu>(
					imuTopic, qos_imu,
					std::bind(&InertialIntegration::imuIntegrator, this, std::placeholders::_1),
					imuOpt);
			subOdometry = create_subscription<nav_msgs::msg::Odometry>(
					"lio_sam/mapping/odometry_incremental", qos,
					std::bind(&InertialIntegration::odomIntegrator, this, std::placeholders::_1),
					odomOpt);

			// Create Publisher
			pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic+"_incremental", qos_imu);

			// gtsam Init TODO: is this the best way to do this? Seems messy.
			boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
			p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
			p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
			p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
			gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

			priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
			priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
			priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
			correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
			correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
			noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

			imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
			imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization  

		}

		// Helper Functions
		void resetOptimization() {
			gtsam::ISAM2Params optParameters;
			optParameters.relinearizeThreshold = 0.1;
			optParameters.relinearizeSkip = 1;
			optimizer = gtsam::ISAM2(optParameters);
			gtsam::NonlinearFactorGraph newGraphFactors;
			graphFactors = newGraphFactors;
			gtsam::Values NewGraphValues;
			graphValues = NewGraphValues;
			return;
		}
		void resetParams() { // this resets (almost) everything
			lastImuT_imu = -1;
			doneFirstOpt = false;
			systemInitialized = false;
			return;
		}
		bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
		{
			Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
			if (vel.norm() > 30)
			{
				RCLCPP_WARN(get_logger(), "IMU Integration Error: Large velocity");
				return true;
			}

			Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
			Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
			if (ba.norm() > 1.0 || bg.norm() > 1.0)
			{
				RCLCPP_WARN(get_logger(), "IMU Integration Error: Large bias");
				return true;
			}

			return false;
		}
		void sysInit(double currentCorrectionTime, gtsam::Pose3 lidarPose) {
			resetOptimization();

            // Pop old IMU messages until we're at the current correction time
            while (!imuQueOpt.empty())
            {
                if (stamp2Sec(imuQueOpt.front().header.stamp) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = stamp2Sec(imuQueOpt.front().header.stamp);
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // Init Pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // Init Velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // Init Bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // Add Init Values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // Optimize Once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            return;
		}
		void resetGraph() {
			// Get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // Reset Graph
            resetOptimization();
            // Add Pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // Add Velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // Add Bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // Add Values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // Optimize Once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
		}

		// IMU Process
		void imuIntegration(const sensor_msgs::msg::Imu::SharedPtr imu_raw) {
			std::lock_guard<std::mutex> lock(mtx); // lock this thread
			sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw); // rotates axes based on extrinsics (comes from ParamServer)

			imuQueOpt.push_back(thisImu); // push into queue
			imuQueImu.push_back(thisImu); // push into queue

			if (!doneFirstOpt) // need to optimize first
				return;

			double imuTime = stamp2Sec(thisImu.header.stamp); // corrected IMU time
			double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu); // TODO: imu freq is hardcoded; add to param.yaml

			// Integrate Current IMU Message
			imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
													gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

			// Predict Odometry
			gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

			// Setup Odometry Message (this gets used by ImageProjection AND TransformFusion)
			auto odometry = nav_msgs::msg::Odometry();
			odometry.header.stamp = thisImu.header.stamp;
			odometry.header.frame_id = odometryFrame;
			odometry.child_frame_id = "odom_imu";
			gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
			gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar); // transform IMU pose to LiDAR pose
			
			// Pack Data
			odometry.pose.pose.position.x = lidarPose.translation().x();
			odometry.pose.pose.position.y = lidarPose.translation().y();
			odometry.pose.pose.position.z = lidarPose.translation().z();
			odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
			odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
			odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
			odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
			odometry.twist.twist.linear.x = currentState.velocity().x();
			odometry.twist.twist.linear.y = currentState.velocity().y();
			odometry.twist.twist.linear.z = currentState.velocity().z();
			odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
			odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
			odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
			pubImuOdometry->publish(odometry); // publish
		}

		// Odom Process
		void odomIntegrator(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
			std::lock_guard<std::mutex> lock(mtx); // lock this thread
			double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);

			if (!imuQueOpt.empty()) // we need IMU data to integrate
				return;

			// Pack LiDAR pose
			float p_x = odomMsg->pose.pose.position.x;
			float p_y = odomMsg->pose.pose.position.y;
			float p_z = odomMsg->pose.pose.position.z;
			float r_x = odomMsg->pose.pose.orientation.x;
			float r_y = odomMsg->pose.pose.orientation.y;
			float r_z = odomMsg->pose.pose.orientation.z;
			float r_w = odomMsg->pose.pose.orientation.w;
			bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false; // this tolerance seems... high
			gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

			// Optimize Routine
			
			if (!systemInitialized) { // init
				optInit();
			} else {
				if (key >= 100) // reset graph when iterations are high
					graphReset();

				// Integrate IMU and Optimize
				while (!imuQueOpt.empty()) {
					// pop and integrate imu data that is between two optimizations
					sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
					double imuTime = stamp2Sec(thisImu->header.stamp);
					if (imuTime < currentCorrectionTime - delta_t) {
						double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt); // TODO: hardcoded IMU freq
						imuIntegratorOpt_->integrateMeasurement(
								gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
								gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

						lastImuT_opt = imuTime;
						imuQueOpt.pop_front();
					}
					else
						break;
				}
				// add imu factor to graph
				const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
				gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
				graphFactors.add(imu_factor);
				// add imu bias between factor
				graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
								 gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
				// add pose factor
				gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
				gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
				graphFactors.add(pose_factor);
				// insert predicted values
				gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
				graphValues.insert(X(key), propState_.pose());
				graphValues.insert(V(key), propState_.v());
				graphValues.insert(B(key), prevBias_);
				// optimize
				optimizer.update(graphFactors, graphValues);
				optimizer.update();
				graphFactors.resize(0);
				graphValues.clear();
				// Overwrite the beginning of the preintegration for the next step.
				gtsam::Values result = optimizer.calculateEstimate();
				prevPose_  = result.at<gtsam::Pose3>(X(key));
				prevVel_   = result.at<gtsam::Vector3>(V(key));
				prevState_ = gtsam::NavState(prevPose_, prevVel_);
				prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
				// Reset the optimization preintegration object.
				imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

				// Check Optimization
				if (failureDetection(prevVel_, prevBias_)) {
					resetParams();
					return; // exit callback
				}

				// Re-propagate IMU Odometry
				prevStateOdom = prevState_;
				prevBiasOdom  = prevBias_;
				// first pop imu message older than current correction data
				double lastImuQT = -1;
				while (!imuQueImu.empty() && stamp2Sec(imuQueImu.front().header.stamp) < currentCorrectionTime - delta_t) {
					lastImuQT = stamp2Sec(imuQueImu.front().header.stamp);
					imuQueImu.pop_front();
				}
				// repropogate
				if (!imuQueImu.empty()) {
					// reset bias use the newly optimized bias
					imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
					// integrate imu message from the beginning of this optimization
					for (int i = 0; i < (int)imuQueImu.size(); ++i) {
						sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
						double imuTime = stamp2Sec(thisImu->header.stamp);
						double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

						imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
								gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
						lastImuQT = imuTime;
					}
				}

				++key;
				doneFirstOpt = true;
			}

			return;
		}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv); // ROS2 init

	// This relies on ROS2 *Node* multithreading at the cost of extra publishers and subscribers
	// TODO: might be better to use C++ multithreading to keep everything in memory
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);

	rclcpp:: executors::MultiThreadedExecutor exec; // this also uses ROS2 multithreading, but at the *callback* level. This is ideal.
	exec.add_node(std::make_shared<InertialIntegration>(options)); // add the imu integration node; Would be better if this wasn't its own node.

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inertialIntegration is alive.");
	exec.spin();
	rclcpp::shutdown();
	return 0;
}
