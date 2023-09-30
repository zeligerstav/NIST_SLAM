#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"

// Generic Laser PointCloud Structure
struct PointXYZIRT {
	PCL_ADD_POINT4D
	PCL_ADD_INTENSITY;
	uint16_t ring;
	float time;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW	
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Ouster Laser Implementation
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

const int queueLength = 2000; // global constant queue length

// Node definition
class ImageProjection : public ParamNode { // extends ParamNode
	private:
		// Fields
		std::mutex imuLock; // thread locker for IMU process
		std::mutex odoLock; // thread locker for odometry process
		bool firstPointFlag;
		int imuPointerCur;
		int deskewFlag; // TODO: shouldn't this always be true?
		double *imuTime = new double[queueLength]; // this isn't really good practice
		double *imuRotX = new double[queueLength];
		double *imuRotY = new double[queueLength];
		double *imuRotZ = new double[queueLength];
		double timeScanCur;
		double timeScanEnd;
		float odomIncreX;
		float odomIncreY;
		float odomIncreZ;
		vector<int> columnIdnCountVec;
		
		// Standard Queues
		std::deque<sensor_msgs::msg::Imu> imuQueue;
		std::deque<nav_msgs::msg::Odometry> odomQueue;
		std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;

		// Messages
		sensor_msgs::msg::PointCloud2 currentCloudMsg;
		lio_sam::msg::CloudInfo cloudInfo;
		std_msgs::msg::Header cloudHeader;

		// Custom Objects
		Eigen::Affine3f transStartInverse;
		cv::Mat rangeMat;
		
		// PC2 Pointers
		pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn; // use this structure for calculations
		pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn; // use this structure to convert Ouster struct to generic struct
		pcl::PointCloud<PointType>::Ptr fullCloud;
		pcl::PointCloud<PointType>::Ptr extractedCloud;

		// Subscriber and Publisher Pointers
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud; // raw point cloud data
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu; // raw imu data
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom; // odom data from InertialIntegration
		rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
		rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
		rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud; // Octomap uses this
		rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo; // featureExtration uses this

		// Constructor
		ImageProjection(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_imageProjection", options), deskewFlag(0) {
			// Init
			ringFlag = 1; // we should always have a ring field
			deskewFlag = 1; // we always want deskew

			// Setup Callbacks
			callbackGroupLidar = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			auto lidarOpt = rclcpp::SubscriptionOptions();
			lidarOpt.callback_group = callbackGroupLidar;
			auto imuOpt = rclcpp::SubscriptionOptions();
			imuOpt.callback_group = callbackGroupImu;
			auto odomOpt = rclcpp::SubscriptionOptions();
			odomOpt.callback_group = callbackGroupOdom;

			// Subscribers
			// imu and odom callbacks just add messages to the queue until a PointCloud2 is receieved
			// cloudCallback does all the work here
			subImu = create_subscription<sensor_msgs::msg::Imu>(
					imuTopic, qos_imu,
					std::bind(&ImageProjection::imuCallback, this, std::placeholders::_1),
					imuOpt);
			subOdom = create_subscription<nav_msgs::msg::Odometry>(
					odomTopic + "_incremental", qos_imu,
					std::bind(&ImageProjection::odomCallback, this, std::placeholders::_1),
					odomOpt);
			subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
					pointCloudTopic, qos_lidar,
					std::bind(&ImageProjection::cloudCallback, this, std::placeholders::_1),
					lidarOpt);

			// Publishers
			pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
			pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>("lio_sam/deskew/cloud_info", qos);

			// Init
			allocateMemory();
			resetParameters();
			pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
		}

		// Helper Functions
		void allocateMemory() {
			laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
			tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
			fullCloud.reset(new pcl::PointCloud<PointType>());
			extractedCloud.reset(new pcl::PointCloud<PointType>());

			fullCloud->points.resize(N_SCAN*Horizon_SCAN);

			cloudInfo.start_ring_index.assign(N_SCAN, 0);
			cloudInfo.end_ring_index.assign(N_SCAN, 0);

			cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
			cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);

			resetParameters();
		}
		void resetParameters() {
			laserCloudIn->clear();
			extractedCloud->clear();
			// reset range matrix for range image projection
			rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

			imuPointerCur = 0;
			firstPointFlag = true;
			odomDeskewFlag = false;

			for (int i = 0; i < queueLength; ++i)
			{
				imuTime[i] = 0;
				imuRotX[i] = 0;
				imuRotY[i] = 0;
				imuRotZ[i] = 0;
			}
		}
		~ImageProjection(){} // TODO: what is this for??
		bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & laserCloudMsg) {
			// Cache
			cloudQueue.push_back(*laserCloudMsg);
			if (cloudQueue.size() <= 2) // nothing to cache
				return false

			// Convert to Generic Representation
			currentCloudMsg = std::move(cloudQueue.front());
			cloudQueue.pop_front();
			pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudin);
			laserCloudIn->points.resize(tmpOusterCloudIn->size()); // resize
			laserCloudIn->is_dense = tmpOusterCloudIn->is_dense; // dense flag; probably not needed
			for (size_t i = 0; i < tmpOusterCloudIn->size(); i++) { // move over each point in ouster cloud
				auto &src = tmpOusterCloudIn->points[i];
				auto &dst = laserCloudIn->points[i];
				dst.x = src.x;
				dst.y = src.y;
				dst.z = src.z;
				dst.intensity = src.intensity;
				dst.ring = src.ring;
				dst.time = src.t * 1e-9f;
			}

			// Get Timestamp
			cloudHeader = currentCloudMsg.header;
			timeScanCur = stamp2Sec(cloudHeader.stamp);
			timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

			return true;
		}
		bool deskewInfo() {
			std::lock_guard<std::mutex> lock1(imuLock); // lock IMU
			std::lock_guard<std::mutex> lock2(odoLock)l // lock Odom TODO: why do we need two locks?

			bool deskew = false;
			
			if (imuQueue.empty() || stamp2Sec(imuQueue.front().header.stamp) > timeScanCur || stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd) {
				deskew = true;
				imuDeskewInfo();
				odomDeskewInfo();
			} else {
				RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
			}

			return deskew;
		}
		void imuDeskewInfo() {
			cloudInfo.imu_available = false;

			while (!imuQueue.empty()) { // pop IMU messages until we're at the current LiDAR message
				if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01) // hard coded value here
					imuQueue.pop_front();
				else
					break;
			}

			if (imuQueue.empty()) // make sure we have enough IMU messages
				return; // exit

			imuPointerCur = 0;

			for (int i = 0; i < (int) imuQueue.size(); ++i) {
				sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
				double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

				// Normally, we'd get an orientation estimate here, but we don't have one
				// TODO: should we estimate orientation here? The example doesn't do it until mapOptimization
				if (currentImuTime > timeScanEnd + 0.01) // hardcoded value here
					break;

				if (imuPointerCur == 0) { // init IMU rotation
					imuRotX[0] = 0;
					imuRotY[0] = 0;
					imuRotZ[0] = 0;
					imuTime[0] = currentImuTime;
					++imuPointerCur;
					continue; // don't think this is necessary?
				}

				double angular_x, angular_y, angular_z; // angular velocities
				imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);
		
				double timeDiff = currentImuTime - imuTime[imuPointerCur - 1]; // this is where it's important to have a dt for the LiDAR
				imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
				imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
				imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
				imuTime[imuPointerCur] = currentImuTime;
				++imuPointerCur;
			}

			--imuPointerCur;

			if (imuPointerCur > 0)
				cloudInfo.imu_available = true;

			return;
		}
		void odomDeskewInfo() {
			cloudInfo.odom_available = false;

			while (!odomQueue.empty()) { // pop until we're at the most recent odom message
				if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01) // hardcoded value
					odomQueue.pop_front();
				else
					break;
			}

			if (odomQueue.empty() || stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
				return;

			nav_msgs::msg:Odometry startOdomMsg;

			for (int i = 0; i < (int) odomQueue.size(); ++i) { // find start of odom message for this scan
				startOdomMsg = odomQueue[i];
				if (stamp2Sec(startOdomMsg.header.stamp) >= timeScanCur)
					break;
			}

			// TF stuff
			tf2::Quaternion orientation;
			tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

			double roll, pitch, yaw;
			tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw); // TODO: will this not just be zero for us every time?

			// Initial Guess for MapOptimization
			cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
			cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
			cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
			cloudInfo.initial_guess_roll = roll;
			cloudInfo.initial_guess_pitch = pitch;
			cloudInfo.initial_guess_yaw = yaw;

			cloudInfo.odom_available = true;

			odomDeskewFlag = false;
			if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)
				return;
			nav_msgs::msg::Odometry endOdomMsg;

			for (int i = 0; i < (int) odomQueue.size(); ++i) {
				endOdomMsg = odomQueue[i];
				if (stamp2Sec(endOdomMsg.header.stamp) >= timeScanEnd)
					break;
			}

			if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0]))) // check covariance
				return;

			// Eigen and TF
			Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);
			tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
			tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
			Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);
			Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

			// Translations
			float rollIncre, pitchIncre, yawIncre;
			pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

			odomDeskewFlag = true;
		}
		void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
			*rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

			int imuPointerFront = 0;
			while (imuPointerFront < imuPointerCur)
			{
				if (pointTime < imuTime[imuPointerFront])
					break;
				++imuPointerFront;
			}

			if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
			{
				*rotXCur = imuRotX[imuPointerFront];
				*rotYCur = imuRotY[imuPointerFront];
				*rotZCur = imuRotZ[imuPointerFront];
			} else {
				int imuPointerBack = imuPointerFront - 1;
				double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
				double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
				*rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
				*rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
				*rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
			}
		}
		void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {
			if (cloudInfo.odomAvailable == false)
				return;
			float ratio = relTime / (timeScanEnd - timeScanCur);

			*posXCur = ratio * odomIncreX;
			*posYCur = ratio * odomIncreY;
			*posZCur = ratio * odomIncreZ;
		}
		PointType deskewPoint(PointType *point, double relTime) {
			if (cloudInfo.imu_available == false)
				return *point;

			double pointTime = timeScanCur + relTime;

			// Find position and rotation
			float rotXCur, rotYCur, rotZCur;
			findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
			float posXCur, posYCur, posZCur;
			findPosition(relTime, &posXCur, &posYCur, &posZCur);

			if (firstPointFlag == true) {
				transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
				firstPointFlag = false;
			}

			// transform points to start
			Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
			Eigen::Affine3f transBt = transStartInverse * transFinal;

			PointType newPoint;
			newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
			newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
			newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
			newPoint.intensity = point->intensity;

			return newPoint;
		}
		void projectPointCloud() {
			int cloudSize = laserCloudIn->points.size();

			// Range Image Projection
			for (int i = 0; i < cloudSize; ++i) {
				PointType thisPoint;
				thisPoint.x = laserCloudIn->points[i].x;
				thisPoint.y = laserCloudIn->points[i].y;
				thisPoint.z = laserCloudIn->points[i].z;
				thisPoint.intensity = laserCloudIn->points[i].intensity;

				float range = pointDistance(thisPoint);
				int rowIdn = laserCloudIn->points[i].ring;
				if (range < lidarMinRange || range > lidarMaxRange || rowIdn < 0 || rowIdn >= N_SCAN || rowIdn % downsampleRate != 0)
					continue; // skip this iteration

				float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_IP;
				static float ang_res_x = 360.0 / float(Horizon_SCAN);
				int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + HorizonSCAN / 2;
				if (columnIdn >= Horizon_SCAN)
					columnIdn -= Horizon_SCAN;

				if (columnIdn < 0 || columnIdn >= Horizon_SCAN || rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
					continue;

				thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
				rangeMat.at<float>(rowIdn, columnIdn) = range;

				int index = columnIdn + rowIdn * Horizon_SCAN;
				fullCloud->points[index] - thisPoint;
			}
		}
		void cloudExtraction() {
			int count = 0;

			// Extract segmented cloud for LiDAR odometry
			for (int i = 0; i < N_SCAN; ++i) {
				cloudInfo.start_ring_index[i] = count + 4; // why +4?
				for (int j = 0; j < Horizon_SCAN; ++j) {
					if (rangeMat.at<float>(i,j) != FLT_MAX) {
						// mark the points' column index for marking occlusion later
						cloudInfo.point_col_ind[count] = j;
						// save range info
						cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
						// save extracted cloud
						extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
						// size of extracted cloud
						++count;
					}
				}
				cloudInfo.end_ring_index[i] = count - 6;
			}
		}
		void publishClouds() { // for Octomap
			cloudInfo.header = cloudHeader;
			cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, exxtractedCloud, cloudHeader.stamp, lidarFrame);
			pubLaserCloudInfo->publish(cloudInfo);
		}

		// IMU Callback
		void imuCallback(const sensor_msgs::msg:Imu::SharedPtr imuMsg) {
			sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg); // from ParamNode
			std::lock_guard<std::mutex> lock1(imuLock);
			imuQueue.push_back(thisImu);
		}

		// Odom Callback
		void odomCallback(const nav_msgs::msg:Odometry::SharedPtr odometryMsg) {
			std::lock_guard<std::mutex> lock2(odoLock);
			odomQueue.push_back(*odometryMsg);
		}

		// Cloud Callback
		void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
			if (cachePointCloud(laserCloudMsg) && deskewInfo()) {
				projectPointCloud();
				cloudExtracion();
				publishClouds();
				resetParameters();
			}
		}
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv); // init ROS2

	// ROS2 Node Multithreading
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	
	rclcpp::executors::MultiThreadedExecutor exec; // callback multithreading
	exec.add_node(std::make_shared<ImageProjection>(options)); // add ImageProjection Node

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ImageProjection is alive.");
	exec.spin();
	rclcpp::shutdown();
	return 0; 
}
