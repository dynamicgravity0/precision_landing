#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <optional>

using namespace std::chrono_literals;

// =====================================================================
// Reusable PID with anti-windup + low-pass filtered derivative.
// Unchanged — rotation is a coordinate transform on the ERROR fed in,
// not something the PID itself needs to know about. Magnitude-preserving
// rotations mean these gains stay valid whether the error is expressed
// in ENU or body coordinates.
// =====================================================================
class PIDController
{
public:
    PIDController() = default;

    PIDController(double kp, double ki, double kd, double max_output)
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output),
          integral_(0.0), prev_error_(0.0), filtered_derivative_(0.0), first_run_(true) {}

    double compute(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;

        double p_term = kp_ * error;

        integral_ += error * dt;
        if (ki_ > 0.0) {
            const double integral_limit = (max_output_ * 0.15) / ki_;
            integral_ = std::clamp(integral_, -integral_limit, integral_limit);
        }
        double i_term = ki_ * integral_;

        double d_term = 0.0;
        if (!first_run_) {
            double raw_derivative = (error - prev_error_) / dt;
            constexpr double alpha = 0.15;
            filtered_derivative_ = alpha * raw_derivative + (1.0 - alpha) * filtered_derivative_;
            d_term = kd_ * filtered_derivative_;
        } else {
            first_run_ = false;
        }
        prev_error_ = error;

        double output = p_term + i_term + d_term;
        return std::clamp(output, -max_output_, max_output_);
    }

    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
        filtered_derivative_ = 0.0;
        first_run_ = true;
    }

private:
    double kp_ = 0, ki_ = 0, kd_ = 0, max_output_ = 0;
    double integral_ = 0, prev_error_ = 0, filtered_derivative_ = 0;
    bool first_run_ = true;
};

// =====================================================================
// lat/lon -> local ENU meters relative to an origin. Equirectangular
// approximation, fine at SITL-scale distances. Unchanged.
// =====================================================================
struct LocalEnu { double east = 0.0, north = 0.0; };

LocalEnu geodeticToLocalEnu(double lat_deg, double lon_deg,
                             double origin_lat_deg, double origin_lon_deg)
{
    constexpr double kEarthRadius = 6378137.0;
    constexpr double kDegToRad = M_PI / 180.0;

    const double origin_lat_rad = origin_lat_deg * kDegToRad;
    const double dlat = (lat_deg - origin_lat_deg) * kDegToRad;
    const double dlon = (lon_deg - origin_lon_deg) * kDegToRad;

    LocalEnu enu;
    enu.north = dlat * kEarthRadius;
    enu.east  = dlon * kEarthRadius * std::cos(origin_lat_rad);
    return enu;
}

enum class LandingState {
    GPS_SEARCH,
    VISION_TRACKING,
    VISION_DESCENT,
    TOUCHDOWN
};

class LandingManager : public rclcpp::Node
{
public:
    LandingManager() : Node("landing_manager")
    {
        this->declare_parameter<int>("target_aruco_id", 0);
        this->declare_parameter<double>("rover_telemetry_max_age_sec", 1.0);
        this->declare_parameter<double>("aruco_max_age_sec", 0.5);
        this->declare_parameter<double>("descent_reacquire_grace_sec", 2.5);
        this->declare_parameter<double>("landed_altitude_threshold_m", 0.15);
        this->declare_parameter<int>("acquisition_consecutive_detections", 5);

        target_aruco_id_ = this->get_parameter("target_aruco_id").as_int();
        rover_telemetry_max_age_ = this->get_parameter("rover_telemetry_max_age_sec").as_double();
        aruco_max_age_ = this->get_parameter("aruco_max_age_sec").as_double();
        descent_reacquire_grace_ = this->get_parameter("descent_reacquire_grace_sec").as_double();
        landed_altitude_threshold_ = this->get_parameter("landed_altitude_threshold_m").as_double();
        acquisition_required_hits_ = this->get_parameter("acquisition_consecutive_detections").as_int();

        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/uav/rover_relative_pose", 10,
            std::bind(&LandingManager::aruco_callback, this, std::placeholders::_1));

        // BEST_EFFORT/VOLATILE/depth 5 per AP_DDS_Topic_Table.h (LOCAL_POSE_PUB).
        drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ap/v1/pose/filtered", rclcpp::SensorDataQoS(),
            std::bind(&LandingManager::drone_pose_callback, this, std::placeholders::_1));

        rover_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/ap/v2/navsat", rclcpp::SensorDataQoS(),
            std::bind(&LandingManager::rover_gps_callback, this, std::placeholders::_1));

        rover_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/ap/v2/twist/filtered", rclcpp::SensorDataQoS(),
            std::bind(&LandingManager::rover_vel_callback, this, std::placeholders::_1));

        // Rover's own orientation — used ONLY to yaw the drone to face/match
        // the rover's heading during the vision states, so the drone stops
        // oscillating when the rover is turning (e.g. driving in a circle).
        rover_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ap/v2/pose/filtered", rclcpp::SensorDataQoS(),
            std::bind(&LandingManager::rover_pose_callback, this, std::placeholders::_1));

        origin_sub_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
            "/ap/v1/gps_global_origin/filtered", rclcpp::SensorDataQoS(),
            std::bind(&LandingManager::origin_callback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/ap/v1/cmd_vel", 10);

        // GPS_SEARCH PID operates directly on ENU error (east/north) — no
        // rotation applied on our side. See rationale in prose.
        gps_pid_east_  = PIDController(0.25, 0.05, 0.1, 1.0);
        gps_pid_north_ = PIDController(0.25, 0.05, 0.1, 1.0);

        // Vision PID operates on body-frame marker offsets, unchanged.
        vision_pid_x_ = PIDController(0.25, 0.04, 0.05, 1.0);
        vision_pid_y_ = PIDController(0.25, 0.04, 0.05, 1.0);
        vision_pid_z_ = PIDController(0.12, 0.01, 0.02, 0.30);

        // Yaw-rate PID: drives drone_yaw toward rover_yaw. No integral term
        // (Ki=0) — this is tracking a continuously-moving target, not
        // correcting a fixed setpoint, so windup risk isn't worth it here.
        // max_output is a yaw-RATE clamp (rad/s), not an angle.
        this->declare_parameter<double>("yaw_kp", 1.0);
        this->declare_parameter<double>("yaw_kd", 0.1);
        this->declare_parameter<double>("max_yaw_rate_rad_s", 0.5);
        const double yaw_kp = this->get_parameter("yaw_kp").as_double();
        const double yaw_kd = this->get_parameter("yaw_kd").as_double();
        const double max_yaw_rate = this->get_parameter("max_yaw_rate_rad_s").as_double();
        yaw_pid_ = PIDController(yaw_kp, 0.0, yaw_kd, max_yaw_rate);

        center_tolerance_ = 0.20;
        descent_hysteresis_ = 1.5;
        land_altitude_ = 0.45;
        touchdown_descent_rate_ = 0.25;

        current_state_ = LandingState::GPS_SEARCH;
        last_aruco_time_ = this->now();
        last_loop_time_ = this->now();

        control_timer_ = this->create_wall_timer(50ms, std::bind(&LandingManager::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Landing manager started in GPS_SEARCH.");
    }

private:
    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // TODO: gate on marker_id / pose_quality once aruco_detector_node
        // exposes them — message currently only carries a bare pose.

        // Reacquiring after a hold-gap (see control_loop's DESCENT/TOUCHDOWN
        // hold logic): the vision PIDs' prev_error_/derivative state predates
        // the gap. Feeding a fresh post-gap reading against that stale
        // state over a normal ~50ms tick would look like an instantaneous
        // jump and produce a derivative spike. Reset instead of carrying it.
        const double stale_for = (this->now() - last_aruco_time_).seconds();
        if ((current_state_ == LandingState::VISION_DESCENT ||
             current_state_ == LandingState::TOUCHDOWN) && stale_for > aruco_max_age_) {
            vision_pid_x_.reset();
            vision_pid_y_.reset();
            vision_pid_z_.reset();
            RCLCPP_INFO(this->get_logger(), "ArUco reacquired after %.2fs gap — vision PIDs reset.", stale_for);
        }

        marker_position_body_x_ = msg->pose.position.x;
        marker_position_body_y_ = msg->pose.position.y;
        marker_position_body_z_ = msg->pose.position.z;
        last_aruco_time_ = this->now();

        if (current_state_ == LandingState::GPS_SEARCH) {
            consecutive_detections_++;
            if (consecutive_detections_ >= acquisition_required_hits_) {
                current_state_ = LandingState::VISION_TRACKING;
                gps_pid_east_.reset();
                gps_pid_north_.reset();
                RCLCPP_INFO(this->get_logger(),
                    "ArUco acquired (%d consecutive detections). GPS_SEARCH -> VISION_TRACKING.",
                    consecutive_detections_);
            }
        } else {
            consecutive_detections_ = acquisition_required_hits_;
        }
    }

    void drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Verified: position.x/y on this topic are ENU east/north despite
        // header.frame_id being hardcoded/mislabeled "base_link" by this
        // AP_DDS build (confirmed by cross-checking Δposition/Δt against
        // /ap/v2/twist/filtered on the rover's matching topic).
        drone_position_enu_east_  = msg->pose.position.x;
        drone_position_enu_north_ = msg->pose.position.y;
        drone_position_enu_up_    = msg->pose.position.z; // vision-independent altitude — used to detect landed in TOUCHDOWN
        // Still needed — but ONLY for rotating rover FF into body frame
        // during the vision states (see roverVelocityFeedforwardBody()).
        // GPS_SEARCH no longer uses this at all.
        drone_yaw_rad_ = tf2::getYaw(msg->pose.orientation);
        have_drone_pose_ = true;
    }

    void rover_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        rover_lat_ = msg->latitude;
        rover_lon_ = msg->longitude;
        last_rover_gps_time_ = this->now();
        have_rover_gps_ = true;
    }

    void rover_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // Verified ENU (east/north) — see prior derivative cross-check.
        // Never rotated by rover yaw.
        rover_velocity_enu_east_  = msg->twist.linear.x;
        rover_velocity_enu_north_ = msg->twist.linear.y;
        last_rover_vel_time_ = this->now();
        have_rover_vel_ = true;
    }

    void rover_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        rover_yaw_rad_ = tf2::getYaw(msg->pose.orientation);
        last_rover_pose_time_ = this->now();
        have_rover_pose_ = true;
    }

    void origin_callback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg)
    {
        origin_lat_ = msg->position.latitude;
        origin_lon_ = msg->position.longitude;
        have_origin_ = true;
    }

    // ENU -> body(FLU) rotation. Used ONLY to align rover FF with the
    // body-frame vision correction during VISION_*/TOUCHDOWN — NOT used
    // anywhere in GPS_SEARCH anymore (that was the double-rotation bug:
    // rotating an ENU quantity into body using our own yaw, then
    // publishing as base_link, causing AP's body_to_earth() to rotate it
    // AGAIN using its own AHRS yaw).
    void rotateEnuToBody(double east, double north, double yaw_rad, double &body_x_fwd, double &body_y_left) const
    {
        const double c = std::cos(yaw_rad);
        const double s = std::sin(yaw_rad);
        body_x_fwd  =  c * east + s * north;
        body_y_left = -s * east + c * north;
    }

    // Raw ENU rover feedforward, staleness-checked, NO rotation. Used
    // directly by GPS_SEARCH (both terms already ENU there).
    std::pair<double, double> roverVelocityFeedforwardEnu() const
    {
        if (!have_rover_vel_) return {0.0, 0.0};
        const double age = (this->now() - last_rover_vel_time_).seconds();
        if (age > rover_telemetry_max_age_) return {0.0, 0.0};
        return {rover_velocity_enu_east_, rover_velocity_enu_north_};
    }

    // Rover feedforward rotated into body frame — used ONLY by the vision
    // states, where it must be aligned with the body-frame vision
    // correction before both are summed into one base_link command.
    std::pair<double, double> roverVelocityFeedforwardBody() const
    {
        auto [east, north] = roverVelocityFeedforwardEnu();
        if (!have_drone_pose_) return {0.0, 0.0};
        double fx = 0.0, fy = 0.0;
        rotateEnuToBody(east, north, drone_yaw_rad_, fx, fy);
        return {fx, fy};
    }

    static double wrapAngle(double a)
    {
        // Wrap to [-pi, pi] via atan2 — handles the wraparound cleanly
        // without a branchy modulo (needed since a plain difference of two
        // yaws can land outside [-pi,pi], e.g. rover at +170°, drone at
        // -170°, true error is 20°, not -340°).
        return std::atan2(std::sin(a), std::cos(a));
    }

    // Yaw-rate command to align drone heading with rover heading. Returns
    // 0.0 if rover attitude is unavailable or stale — no yaw motion rather
    // than yawing toward a guess.
    double yawRateCommand(double dt)
    {
        if (!have_rover_pose_ || !have_drone_pose_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[YAW] no command — have_rover_pose=%d have_drone_pose=%d",
                have_rover_pose_, have_drone_pose_);
            return 0.0;
        }
        const double age = (this->now() - last_rover_pose_time_).seconds();
        if (age > rover_telemetry_max_age_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[YAW] no command — rover pose stale (%.2fs > %.2fs)", age, rover_telemetry_max_age_);
            return 0.0;
        }

        const double yaw_error = wrapAngle(rover_yaw_rad_ - drone_yaw_rad_);
        const double yaw_rate_cmd = yaw_pid_.compute(yaw_error, dt);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[YAW] rover_yaw=%.1fdeg drone_yaw=%.1fdeg error=%.1fdeg -> angular.z=%.3f rad/s (AP will negate this)",
            rover_yaw_rad_ * 180.0 / M_PI, drone_yaw_rad_ * 180.0 / M_PI,
            yaw_error * 180.0 / M_PI, yaw_rate_cmd);

        return yaw_rate_cmd;
    }

    void control_loop()
    {
        auto now = this->now();
        double dt = (now - last_loop_time_).seconds();
        last_loop_time_ = now;
        if (dt <= 0.0) return;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = now;
        cmd.header.frame_id = "base_link"; // default for vision states; GPS_SEARCH overrides to "map" below

        const bool aruco_fresh = (now - last_aruco_time_).seconds() <= aruco_max_age_;
        const double aruco_stale_for = (now - last_aruco_time_).seconds();

        // Loss from VISION_TRACKING: immediate fallback to GPS_SEARCH is fine
        // here — higher altitude/farther out, and re-acquiring via the
        // rover's GPS position is a reasonable coarse recovery.
        if (current_state_ == LandingState::VISION_TRACKING && !aruco_fresh)
        {
            current_state_ = LandingState::GPS_SEARCH;
            consecutive_detections_ = 0;
            vision_pid_x_.reset();
            vision_pid_y_.reset();
            vision_pid_z_.reset();
            RCLCPP_WARN(this->get_logger(), "ArUco lost during TRACKING (stale > %.2fs). Reverting to GPS_SEARCH.", aruco_max_age_);
        }

        // Loss from VISION_DESCENT/TOUCHDOWN: do NOT immediately fall back to
        // GPS_SEARCH. The rover's GPS antenna and the ArUco marker are at
        // different physical points on the rover — flying toward the GPS
        // position this close in pulls the drone away from the marker,
        // Loss from VISION_DESCENT only: the rover's GPS antenna and the
        // ArUco marker are at different physical points on the rover —
        // flying toward the GPS position this close in pulls the drone
        // away from the marker, re-triggering acquisition, pulling it
        // back, losing the marker again — an oscillating loop between two
        // different targets. Hover (zero velocity, PIDs paused, not fed
        // the frozen marker value which would wind up on stale data) for
        // a grace window before escalating to GPS_SEARCH.
        //
        // TOUCHDOWN is deliberately excluded here (see below) — vision
        // loss that close to the ground is a persistent FOV limit, not a
        // transient blip; waiting for it to clear just delays an
        // inevitable, unhelpful trip back out to GPS_SEARCH while the
        // rover keeps moving. TOUCHDOWN instead commits to a blind final
        // descent using the drone's own altitude, not vision.
        if (current_state_ == LandingState::VISION_DESCENT && !aruco_fresh)
        {
            if (aruco_stale_for <= descent_reacquire_grace_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "ArUco lost during descent (%.2fs, grace=%.2fs) — holding position, not falling back to GPS.",
                    aruco_stale_for, descent_reacquire_grace_);
                cmd.header.frame_id = "base_link";
                cmd.twist.linear.x = 0.0;
                cmd.twist.linear.y = 0.0;
                cmd.twist.linear.z = 0.0; // hold altitude too — don't blind-descend without visual confirmation
                cmd.twist.angular.z = yawRateCommand(dt); // still track rover heading while holding position
                vel_pub_->publish(cmd);
                return; // skip the switch entirely this tick; PIDs untouched
            } else {
                current_state_ = LandingState::GPS_SEARCH;
                consecutive_detections_ = 0;
                vision_pid_x_.reset();
                vision_pid_y_.reset();
                vision_pid_z_.reset();
                RCLCPP_WARN(this->get_logger(),
                    "ArUco not reacquired within grace window (%.2fs). Escalating to GPS_SEARCH.",
                    descent_reacquire_grace_);
            }
        }

        switch (current_state_) {
            case LandingState::GPS_SEARCH: {
                cmd.header.frame_id = "map"; // ENU command — AP does the ONE necessary ENU->NED conversion itself

                if (!have_origin_ || !have_rover_gps_ || !have_drone_pose_) {
                    cmd.twist.angular.z = yawRateCommand(dt);
                    vel_pub_->publish(cmd);
                    return;
                }
                const double gps_age = (now - last_rover_gps_time_).seconds();
                if (gps_age > rover_telemetry_max_age_) {
                    cmd.twist.angular.z = yawRateCommand(dt);
                    vel_pub_->publish(cmd);
                    return;
                }

                LocalEnu rover_enu = geodeticToLocalEnu(rover_lat_, rover_lon_, origin_lat_, origin_lon_);
                const double error_east  = rover_enu.east  - drone_position_enu_east_;
                const double error_north = rover_enu.north - drone_position_enu_north_;

                // PID directly on ENU error — no rotation.
                const double pid_east  = gps_pid_east_.compute(error_east, dt);
                const double pid_north = gps_pid_north_.compute(error_north, dt);

                auto [ff_east, ff_north] = roverVelocityFeedforwardEnu();

                const double combined_east  = pid_east  + ff_east;
                const double combined_north = pid_north + ff_north;

                cmd.twist.linear.x = combined_east;   // frame_id="map": x=East per handle_velocity_control()
                cmd.twist.linear.y = combined_north;  // y=North
                cmd.twist.linear.z = 0.0;
                cmd.twist.angular.z = yawRateCommand(dt); // yaw rate handling is frame-independent in AP_DDS — safe alongside "map" linear frame

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[GPS_SEARCH] drone_enu(%.2f,%.2f) Rover_enu(%.2f,%.2f) Err_enu(%.2f,%.2f) "
                    "pid_enu(%.2f,%.2f) ff_enu(%.2f,%.2f) combined_enu(%.2f,%.2f) "
                    "frame_id=%s published(x=%.2f,y=%.2f,z=%.2f)",
                    drone_position_enu_east_, drone_position_enu_north_,
                    rover_enu.east, rover_enu.north,
                    error_east, error_north,
                    pid_east, pid_north,
                    ff_east, ff_north,
                    combined_east, combined_north,
                    cmd.header.frame_id.c_str(),
                    cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z);

                vel_pub_->publish(cmd);
                break;
            }

            case LandingState::VISION_TRACKING: {
                const double error_horizontal = std::sqrt(
                    marker_position_body_x_ * marker_position_body_x_ +
                    marker_position_body_y_ * marker_position_body_y_);

                const double vis_x = vision_pid_x_.compute(marker_position_body_x_, dt);
                const double vis_y = -vision_pid_y_.compute(marker_position_body_y_, dt);

                auto [ff_x, ff_y] = roverVelocityFeedforwardBody();
                cmd.twist.linear.x = vis_x + ff_x;
                cmd.twist.linear.y = vis_y + ff_y;
                cmd.twist.linear.z = 0.0;
                cmd.twist.angular.z = yawRateCommand(dt);

                if (error_horizontal < center_tolerance_) {
                    current_state_ = LandingState::VISION_DESCENT;
                    RCLCPP_INFO(this->get_logger(), "Centered. VISION_TRACKING -> VISION_DESCENT.");
                }
                vel_pub_->publish(cmd);
                break;
            }

            case LandingState::VISION_DESCENT: {
                const double error_horizontal = std::sqrt(
                    marker_position_body_x_ * marker_position_body_x_ +
                    marker_position_body_y_ * marker_position_body_y_);

                if (error_horizontal > center_tolerance_ * descent_hysteresis_) {
                    current_state_ = LandingState::VISION_TRACKING;
                    RCLCPP_WARN(this->get_logger(), "Drift during descent. VISION_DESCENT -> VISION_TRACKING.");
                    vel_pub_->publish(cmd);
                    break;
                }

                const double vis_x = vision_pid_x_.compute(marker_position_body_x_, dt);
                const double vis_y = -vision_pid_y_.compute(marker_position_body_y_, dt);
                const double vis_z = vision_pid_z_.compute(marker_position_body_z_, dt);

                auto [ff_x, ff_y] = roverVelocityFeedforwardBody();
                cmd.twist.linear.x = vis_x + ff_x;
                cmd.twist.linear.y = vis_y + ff_y;
                cmd.twist.linear.z = vis_z;
                cmd.twist.angular.z = yawRateCommand(dt);

                if (marker_position_body_z_ > -land_altitude_) {
                    current_state_ = LandingState::TOUCHDOWN;
                    RCLCPP_INFO(this->get_logger(), "Altitude threshold met. VISION_DESCENT -> TOUCHDOWN.");
                }
                vel_pub_->publish(cmd);
                break;
            }

            case LandingState::TOUCHDOWN: {
                // Vision-independent landed check FIRST — drone's own
                // altitude from /ap/v1/pose/filtered, not the marker.
                // At this altitude the marker is known to be unreliable
                // (persistent FOV limit, confirmed empirically), so this
                // is the only trustworthy signal to stop on.
                if (have_drone_pose_ && drone_position_enu_up_ <= landed_altitude_threshold_) {
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.linear.y = 0.0;
                    cmd.twist.linear.z = 0.0; // was 0.25 (climb) — contradicted its own "holding zero" log line; fixed
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Landed (altitude %.2fm <= %.2fm threshold) — holding zero command.",
                        drone_position_enu_up_, landed_altitude_threshold_);
                    vel_pub_->publish(cmd);
                    break;
                }

                double hor_x = 0.0, hor_y = 0.0;
                if (aruco_fresh) {
                    hor_x = vision_pid_x_.compute(marker_position_body_x_, dt);
                    hor_y = -vision_pid_y_.compute(marker_position_body_y_, dt);
                    last_touchdown_hor_x_ = hor_x;
                    last_touchdown_hor_y_ = hor_y;
                } else {
                    // Marker unavailable this close in — expected, not an
                    // error. Hold the last computed correction rather than
                    // feeding compute() a frozen error (would windup the
                    // integral) or zeroing outright (drone was already
                    // centered to a few cm at loss time per observed data,
                    // so holding is a safe, conservative choice).
                    hor_x = last_touchdown_hor_x_;
                    hor_y = last_touchdown_hor_y_;
                }

                auto [ff_x, ff_y] = roverVelocityFeedforwardBody();
                cmd.twist.linear.x = hor_x + ff_x;
                cmd.twist.linear.y = hor_y + ff_y;
                cmd.twist.linear.z = -touchdown_descent_rate_;
                cmd.twist.angular.z = yawRateCommand(dt);
                vel_pub_->publish(cmd);
                break;
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rover_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr rover_gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr rover_vel_sub_;
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    LandingState current_state_;
    rclcpp::Time last_loop_time_;
    rclcpp::Time last_aruco_time_;
    rclcpp::Time last_rover_gps_time_;
    rclcpp::Time last_rover_vel_time_;
    int consecutive_detections_ = 0;

    int target_aruco_id_ = 0;
    double rover_telemetry_max_age_ = 1.0;
    double aruco_max_age_ = 0.5;
    double descent_reacquire_grace_ = 2.5;
    double landed_altitude_threshold_ = 0.15;
    int acquisition_required_hits_ = 5;

    PIDController gps_pid_east_, gps_pid_north_;
    PIDController yaw_pid_;
    PIDController vision_pid_x_, vision_pid_y_, vision_pid_z_;
    double center_tolerance_ = 0.20;
    double descent_hysteresis_ = 1.5;
    double land_altitude_ = 0.45;
    double touchdown_descent_rate_ = 0.25;

    double marker_position_body_x_ = 0.0;
    double marker_position_body_y_ = 0.0;
    double marker_position_body_z_ = 0.0;

    bool have_drone_pose_ = false;
    double drone_position_enu_east_ = 0.0;
    double drone_position_enu_north_ = 0.0;
    double drone_position_enu_up_ = 0.0;
    double drone_yaw_rad_ = 0.0;
    double last_touchdown_hor_x_ = 0.0;
    double last_touchdown_hor_y_ = 0.0;

    bool have_origin_ = false;
    double origin_lat_ = 0.0, origin_lon_ = 0.0;
    bool have_rover_gps_ = false;
    double rover_lat_ = 0.0, rover_lon_ = 0.0;
    bool have_rover_vel_ = false;
    double rover_velocity_enu_east_ = 0.0;
    double rover_velocity_enu_north_ = 0.0;
    bool have_rover_pose_ = false;
    double rover_yaw_rad_ = 0.0;
    rclcpp::Time last_rover_pose_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandingManager>());
    rclcpp::shutdown();
    return 0;
}
