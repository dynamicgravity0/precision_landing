#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArucoDetector : public rclcpp::Node
{
public:
    ArucoDetector() : Node("aruco_detector")
    {
        // 1. Subscribe to bridged Gazebo camera topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));

        // 2. Publisher for estimated 3D target pose relative to the UAV
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uav/rover_relative_pose", 10);

        // 2b. TF2 buffer/listener — used to transform the raw camera-frame pose
        //     into base_link, accounting for the camera's 90 deg downward tilt
        //     and -0.05m mount offset defined in the SDF (camera_joint -> camera link).
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. Configure ArUco parameters (5x5 Marker Dictionary)
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
        detector_params_ = cv::aruco::DetectorParameters::create();

        // 4. Define Camera Matrix & Distortion Coefficients
        // 
        camera_matrix_ = (cv::Mat_<double>(3,3) <<
            325.36451235,   0.0, 371.96251418,
              0.0, 326.06372419, 247.26791073,
              0.0,   0.0,   1.0);

        // Gazebo simulation cameras are perfectly ideal, so distortion coefficients are zero
        // Real camera distortion coefficients
	dist_coeffs_ = (cv::Mat_<double>(5, 1) <<
    	    -0.24976676,
     	     0.09927346,
             0.00194119,
             0.00067086,
            -0.03190410);

        // 5. Define physical size of the ArUco marker printed on the rover (in meters)
        marker_length_ = 0.18; // Assumes a 35cm x 35cm square target

        RCLCPP_INFO(this->get_logger(), "3D ArUco Metric Estimator Node Active.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;

        // Scan frame for marker patterns
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, marker_corners, marker_ids, detector_params_, rejected_candidates);

        if (!marker_ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;

            // Pose Estimation: Computes metric vectors using perspective-n-point geometry
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < marker_ids.size(); ++i) {
                // We extract the Translation Vector (tvecs) which represents X, Y, Z offsets in METERS
                cv::Vec3d translation = tvecs[i];
                cv::Vec3d rotation = rvecs[i];

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header = msg->header; // Standard synchronised timestamp
                pose_msg.header.frame_id = "camera_optical_frame";

                // Map physical metric distances (OpenCV Camera Frame: Z is forward, X is right, Y is down)
                pose_msg.pose.position.x = translation[0]; // Left/Right offset in meters
                pose_msg.pose.position.y = translation[1]; // Up/Down offset in meters
                pose_msg.pose.position.z = translation[2]; // Height/Distance to target in meters

                // Convert ArUco axis rotation vector to a unit Quaternion for ROS 2 compatibility
                double angle = cv::norm(rotation);
                if (angle > 1e-6) {
                    cv::Vec3d axis = rotation / angle;
                    pose_msg.pose.orientation.x = axis[0] * sin(angle / 2.0);
                    pose_msg.pose.orientation.y = axis[1] * sin(angle / 2.0);
                    pose_msg.pose.orientation.z = axis[2] * sin(angle / 2.0);
                    pose_msg.pose.orientation.w = cos(angle / 2.0);
                } else {
                    pose_msg.pose.orientation.w = 1.0;
                }

                // Transform from the OpenCV optical frame into base_link. tf2 chains
                // this through two static transforms published in the launch file:
                //   camera_optical_frame -> camera_link  (fixed REP-103 convention rotation)
                //   camera_link -> base_link              (mechanical mount, from the SDF)
                try {
                    geometry_msgs::msg::PoseStamped pose_in_base =
                        tf_buffer_->transform(pose_msg, "base_link", tf2::durationFromSec(0.1));

                    pose_pub_->publish(pose_in_base);

                    RCLCPP_INFO(this->get_logger(),
                        "Target ID %d | Raw(camera_optical) X:%.2f Y:%.2f Z:%.2f | base_link X:%.2f Y:%.2f Z:%.2f",
                        marker_ids[i],
                        translation[0], translation[1], translation[2],
                        pose_in_base.pose.position.x,
                        pose_in_base.pose.position.y,
                        pose_in_base.pose.position.z);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF transform camera_optical_frame -> base_link failed: %s", ex.what());
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double marker_length_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}
