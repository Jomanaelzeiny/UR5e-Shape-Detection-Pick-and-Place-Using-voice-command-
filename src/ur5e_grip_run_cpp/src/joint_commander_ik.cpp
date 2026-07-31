// ============================================================
//  merged_detection_cartesian_node.cpp
// ============================================================

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robotiq_2f_urcap_adapter/action/gripper_command.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

// NOTE: No 'using' alias for GripperCommand — the package defines its own
// 'typedef struct GripperCommand GripperCommand' at global scope which
// collides with any alias we declare here. Use the fully-qualified type
// robotiq_2f_urcap_adapter::action::GripperCommand throughout instead.
namespace robotiq = robotiq_2f_urcap_adapter::action;

static constexpr double HOME_X  = -0.250;
static constexpr double HOME_Y  =  0.110;
static constexpr double HOME_Z  =  1.214;

static constexpr double PLACE_X = -0.367;
static constexpr double PLACE_Y = -0.306;
static constexpr double PLACE_Z =  1.059;

// ── Gripper constants ─────────────────────────────────────────────────────────
// Adjust GRIPPER_CLOSED_POS (0.0–0.8 m) to match your object's width.
static constexpr double GRIPPER_OPEN_POS   = 0.08;   // fully open
static constexpr double GRIPPER_CLOSED_POS = 0.0;   // closed around object — tune this
static constexpr double GRIPPER_EFFORT     = 140;   // max gripping force
// ADD this line with the other constants:
static constexpr double GRIPPER_SPEED = 0.1;   // m/s — within [0.02, 0.15]
// How long to wait at TARGET before closing the gripper (ms)
static constexpr int SETTLE_AT_TARGET_MS = 800;

class MergedDetectionCartesianNode : public rclcpp::Node
{
public:
    MergedDetectionCartesianNode()
    : Node("merged_detection_cartesian_node"),
      target_received_(false),
      busy_(false),
      locked_ori_x_(0.0),
      locked_ori_y_(0.0),
      locked_ori_z_(0.0),
      locked_ori_w_(1.0),
      orientation_captured_(false),
      joint_states_received_(false),
      gripper_available_(false)
    {
        declare_parameter<std::string>("transform_yaml", "camera_to_base.yaml");
        load_transform(get_parameter("transform_yaml").as_string());

        joint_state_sub_ =
            create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                [this](const sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    if (!joint_states_received_ &&
                        (msg->header.stamp.sec > 0 || msg->header.stamp.nanosec > 0))
                    {
                        RCLCPP_INFO(get_logger(),
                            "First valid /joint_states received (stamp %u.%u).",
                            msg->header.stamp.sec, msg->header.stamp.nanosec);
                        joint_states_received_ = true;
                    }
                });

        detection_sub_ =
            create_subscription<geometry_msgs::msg::PointStamped>(
                "/detection/target_coords", 10,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
                {
                    if (busy_.load()) {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Robot busy — ignoring incoming point");
                        return;
                    }

                    Eigen::Vector3d p_cam(msg->point.x, msg->point.y, msg->point.z);
                    Eigen::Vector3d p_base = R_ * p_cam + t_;

                    RCLCPP_INFO(get_logger(),
                        "[CAM ] x=%.3f  y=%.3f  z=%.3f  (frame: %s)",
                        p_cam.x(), p_cam.y(), p_cam.z(),
                        msg->header.frame_id.c_str());
                    RCLCPP_INFO(get_logger(),
                        "[BASE] x=%.3f  y=%.3f  z=%.3f",
                        p_base.x(), p_base.y(), p_base.z());

                    geometry_msgs::msg::PointStamped transformed;
                    transformed.header.frame_id = "base_link";
                    transformed.header.stamp    = msg->header.stamp;
                    transformed.point.x = p_base.x();
                    transformed.point.y = p_base.y();
                    transformed.point.z = p_base.z();

                    {
                        std::lock_guard<std::mutex> lock(target_mutex_);
                        latest_target_   = transformed;
                        target_received_ = true;
                    }
                });

        init_timer_ = create_wall_timer(500ms, [this]()
        {
            init_timer_->cancel();
            initialize_move_group();
            initialize_gripper();
            std::thread([this]() { run_sequence(); }).detach();
        });
    }

private:

    // =========================================================================
    //  Transform loader
    // =========================================================================
    void load_transform(const std::string& yaml_path)
    {
        try
        {
            YAML::Node cfg = YAML::LoadFile(yaml_path);
            auto R_raw = cfg["R"].as<std::vector<std::vector<double>>>();
            auto t_raw = cfg["t"].as<std::vector<double>>();
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R_(i, j) = R_raw[i][j];
            t_ = Eigen::Vector3d(t_raw[0], t_raw[1], t_raw[2]);
            RCLCPP_INFO(get_logger(), "Loaded transform from: %s", yaml_path.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(),
                "Could not load %s (%s) — using hardcoded values.",
                yaml_path.c_str(), e.what());

            R_ << -0.09371117657111058,  0.03363014601003592, -0.9950312701945611,
                   0.9955771888969105,  -0.003514263800089703,-0.09388136608544659,
                  -0.006654046421876756, -0.999428168118793,  -0.03315208043137689;
            t_ = Eigen::Vector3d(
                0.5625612735445096,
                0.20321120475923166,
                1.1039418472110007);
        }

        RCLCPP_INFO(get_logger(),
            "R =\n[%.4f  %.4f  %.4f]\n[%.4f  %.4f  %.4f]\n[%.4f  %.4f  %.4f]",
            R_(0,0), R_(0,1), R_(0,2),
            R_(1,0), R_(1,1), R_(1,2),
            R_(2,0), R_(2,1), R_(2,2));
        RCLCPP_INFO(get_logger(),
            "t = [%.4f  %.4f  %.4f]", t_.x(), t_.y(), t_.z());
    }

    // =========================================================================
    //  MoveIt setup
    // =========================================================================
    void initialize_move_group()
    {
        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "ur_manipulator");

        move_group_->setPlanningTime(15.0);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setPoseReferenceFrame("base_link");
        move_group_->setNumPlanningAttempts(5);

        RCLCPP_INFO(get_logger(), "MoveGroup ready");
    }

    // =========================================================================
    //  Robotiq gripper setup
    // =========================================================================
    void initialize_gripper()
    {
        gripper_client_ =
            rclcpp_action::create_client<robotiq::GripperCommand>(
                this, "/robotiq_2f_urcap_adapter/gripper_command");

        RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");

        if (!gripper_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(get_logger(),
                "Gripper action server not available after 10 s — "
                "gripper commands will be skipped.");
            gripper_available_ = false;
        } else {
            RCLCPP_INFO(get_logger(), "Gripper action server ready.");
            gripper_available_ = true;
        }
    }

    // =========================================================================
    //  Blocking gripper command helper
    //  position : 0.0 (open) → 0.8 (closed), metres
    //  effort   : 0.0 → 1.0 normalised force
    // =========================================================================
    bool set_gripper(double position, double effort,
                     const std::string& label = "")
    {
        if (!gripper_available_) {
            RCLCPP_WARN(get_logger(),
                "Gripper unavailable — skipping [%s]", label.c_str());
            return false;
        }

        auto goal = robotiq::GripperCommand::Goal();
        goal.command.position   = position;
        goal.command.max_effort = effort;
        goal.command.max_speed  = GRIPPER_SPEED;  

        RCLCPP_INFO(get_logger(),
            "Gripper [%s] → pos=%.2f  effort=%.2f",
            label.c_str(), position, effort);

        std::promise<bool> result_promise;
        auto result_future = result_promise.get_future();

        auto opts = rclcpp_action::Client<robotiq::GripperCommand>::SendGoalOptions();
        opts.result_callback =
            [this, &result_promise, label]
            (const rclcpp_action::ClientGoalHandle<robotiq::GripperCommand>::WrappedResult& result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(),
                        "Gripper [%s] succeeded.", label.c_str());
                    result_promise.set_value(true);
                } else {
                    RCLCPP_WARN(get_logger(),
                        "Gripper [%s] failed (code %d).",
                        label.c_str(), static_cast<int>(result.code));
                    result_promise.set_value(false);
                }
            };

        gripper_client_->async_send_goal(goal, opts);

        if (result_future.wait_for(10s) == std::future_status::timeout) {
            RCLCPP_ERROR(get_logger(), "Gripper [%s] timed out!", label.c_str());
            return false;
        }

        return result_future.get();
    }

    // =========================================================================
    //  Orientation capture (with retries)
    // =========================================================================
    void capture_initial_orientation()
    {
        RCLCPP_INFO(get_logger(),
            "Waiting for /joint_states to publish valid data...");
        while (rclcpp::ok() && !joint_states_received_)
        {
            std::this_thread::sleep_for(200ms);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "No valid /joint_states yet — is the robot driver running?");
        }

        RCLCPP_INFO(get_logger(),
            "/joint_states confirmed. Waiting 2 s for MoveIt state monitor to sync...");
        std::this_thread::sleep_for(2000ms);

        constexpr int    max_attempts   = 10;
        constexpr double retry_delay_ms = 500.0;

        for (int attempt = 1; attempt <= max_attempts; ++attempt)
        {
            move_group_->setStartStateToCurrentState();
            std::this_thread::sleep_for(300ms);

            geometry_msgs::msg::Pose current = move_group_->getCurrentPose().pose;

            double norm = std::sqrt(
                std::pow(current.orientation.x, 2) +
                std::pow(current.orientation.y, 2) +
                std::pow(current.orientation.z, 2) +
                std::pow(current.orientation.w, 2));

            RCLCPP_INFO(get_logger(),
                "Attempt %d/%d — pose norm: %.4f | "
                "pos (%.3f, %.3f, %.3f) ori (%.3f %.3f %.3f %.3f)",
                attempt, max_attempts, norm,
                current.position.x, current.position.y, current.position.z,
                current.orientation.x, current.orientation.y,
                current.orientation.z, current.orientation.w);

            if (norm > 0.99 && norm < 1.01)
            {
                locked_ori_x_ = current.orientation.x;
                locked_ori_y_ = current.orientation.y;
                locked_ori_z_ = current.orientation.z;
                locked_ori_w_ = current.orientation.w;
                orientation_captured_ = true;

                RCLCPP_INFO(get_logger(),
                    "Orientation locked after %d attempt(s) (xyzw): "
                    "%.4f  %.4f  %.4f  %.4f",
                    attempt,
                    locked_ori_x_, locked_ori_y_,
                    locked_ori_z_, locked_ori_w_);
                return;
            }

            RCLCPP_WARN(get_logger(),
                "Quaternion norm %.4f is not ~1.0 — retrying in %.0f ms...",
                norm, retry_delay_ms);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(static_cast<int>(retry_delay_ms)));
        }

        RCLCPP_WARN(get_logger(),
            "Could not get valid orientation after %d attempts — using fallback.",
            max_attempts);

        locked_ori_x_ =  1.000;
        locked_ori_y_ =  0.000;
        locked_ori_z_ = -0.008;
        locked_ori_w_ =  0.004;
        orientation_captured_ = true;

        RCLCPP_WARN(get_logger(),
            "Fallback orientation (xyzw): %.4f  %.4f  %.4f  %.4f",
            locked_ori_x_, locked_ori_y_, locked_ori_z_, locked_ori_w_);
    }

    // =========================================================================
    //  Cartesian move to XYZ
    // =========================================================================
    bool move_to_xyz(double x, double y, double z,
                     const std::string& label = "")
    {
        std::vector<double> joints = move_group_->getCurrentJointValues();
        double joint_sum = 0.0;
        for (auto j : joints) joint_sum += std::abs(j);
        if (joint_sum < 0.001) {
            RCLCPP_ERROR(get_logger(),
                "[%s] Joint states are all zero — cannot plan.", label.c_str());
            return false;
        }

        move_group_->setStartStateToCurrentState();
        std::this_thread::sleep_for(300ms);

        geometry_msgs::msg::Pose target;
        target.position.x    = x;
        target.position.y    = y;
        target.position.z    = z;
        target.orientation.x = locked_ori_x_;
        target.orientation.y = locked_ori_y_;
        target.orientation.z = locked_ori_z_;
        target.orientation.w = locked_ori_w_;

        RCLCPP_INFO(get_logger(),
            "Planning [%s] → x=%.3f  y=%.3f  z=%.3f | ori (xyzw): %.3f %.3f %.3f %.3f",
            label.c_str(), x, y, z,
            locked_ori_x_, locked_ori_y_, locked_ori_z_, locked_ori_w_);

        std::vector<geometry_msgs::msg::Pose> waypoints = {target};
        moveit_msgs::msg::RobotTrajectory trajectory;

        double fraction = move_group_->computeCartesianPath(
            waypoints, 0.002, 0.0, trajectory);

        RCLCPP_INFO(get_logger(),
            "[%s] Cartesian path coverage: %.1f%%",
            label.c_str(), fraction * 100.0);

        if (fraction < 0.9) {
            RCLCPP_WARN(get_logger(),
                "[%s] Could not plan full Cartesian path (%.1f%%).",
                label.c_str(), fraction * 100.0);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        RCLCPP_INFO(get_logger(), "[%s] Executing...", label.c_str());
        bool exec_ok = (move_group_->execute(plan) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

        std::this_thread::sleep_for(500ms);

        if (exec_ok)
            RCLCPP_INFO(get_logger(), "[%s] Reached target.", label.c_str());
        else
            RCLCPP_WARN(get_logger(), "[%s] Execution failed.", label.c_str());

        return exec_ok;
    }

    // =========================================================================
    //  Main state machine
    // =========================================================================
    void run_sequence()
    {
        capture_initial_orientation();

        while (rclcpp::ok() && !orientation_captured_)
            std::this_thread::sleep_for(100ms);

        // ── Open gripper at startup ───────────────────────────────────────
        set_gripper(GRIPPER_OPEN_POS, GRIPPER_EFFORT, "OPEN@START");

        RCLCPP_INFO(get_logger(),
            "Ready — waiting for detections on /detection/target_coords");

        while (rclcpp::ok())
        {
            // ── Poll for new target ───────────────────────────────────────
            geometry_msgs::msg::PointStamped target_copy;
            bool has_target = false;
            {
                std::lock_guard<std::mutex> lock(target_mutex_);
                if (target_received_) {
                    target_copy      = latest_target_;
                    target_received_ = false;
                    has_target       = true;
                }
            }

            if (!has_target) {
                std::this_thread::sleep_for(50ms);
                continue;
            }

            busy_.store(true);

            const double tx = target_copy.point.x;
            const double ty = target_copy.point.y;
            const double tz = target_copy.point.z;

            RCLCPP_INFO(get_logger(),
                "=== TARGET(%.3f, %.3f, %.3f) → PLACE → HOME ===",
                tx, ty, tz);

            // ── 1. Make sure gripper is open before approaching ───────────
            std::this_thread::sleep_for(300ms);
            set_gripper(GRIPPER_OPEN_POS, GRIPPER_EFFORT, "OPEN@PRE-PICK");

            move_to_xyz(HOME_X, HOME_Y, HOME_Z, "HOME");

            // ── 2. Move to detected object ────────────────────────────────
            bool reached = move_to_xyz(tx, ty, tz, "TARGET");

            if (!reached) {
                RCLCPP_WARN(get_logger(),
                    "Target unreachable — aborting cycle, returning home.");
                move_to_xyz(HOME_X, HOME_Y, HOME_Z, "HOME");
                set_gripper(GRIPPER_OPEN_POS, GRIPPER_EFFORT, "OPEN@ABORT");
                busy_.store(false);
                continue;
            }

            // ── 3. Settle at target, then close gripper ───────────────────
            RCLCPP_INFO(get_logger(),
                "Reached TARGET — settling for %d ms before gripping...",
                SETTLE_AT_TARGET_MS);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(SETTLE_AT_TARGET_MS));

            set_gripper(GRIPPER_CLOSED_POS, GRIPPER_EFFORT, "CLOSE@TARGET");

            // Brief pause to let the grip stabilise before lifting/moving
            std::this_thread::sleep_for(300ms);
            
            move_to_xyz(HOME_X, HOME_Y, HOME_Z, "HOME");

            // ── 4. Carry object to PLACE (gripper stays closed) ───────────
            RCLCPP_INFO(get_logger(),
                "Gripper closed — moving to PLACE with object.");
            move_to_xyz(PLACE_X, PLACE_Y, PLACE_Z, "PLACE");
            set_gripper(GRIPPER_OPEN_POS, GRIPPER_EFFORT, "OPEN@PLACE");

            // ── 5. Release object at PLACE ────────────────────────────────
            std::this_thread::sleep_for(300ms);

            move_to_xyz(HOME_X, HOME_Y, HOME_Z, "HOME");
            set_gripper(GRIPPER_OPEN_POS, GRIPPER_EFFORT, "OPEN@ABORT");
            

            busy_.store(false);

            RCLCPP_INFO(get_logger(),
                "=== Cycle done — waiting for next detection ===");
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr init_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
        move_group_;

    rclcpp_action::Client<robotiq::GripperCommand>::SharedPtr gripper_client_;
    bool gripper_available_;

    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;

    std::mutex                       target_mutex_;
    geometry_msgs::msg::PointStamped latest_target_;
    std::atomic<bool>                target_received_;
    std::atomic<bool>                busy_;

    double locked_ori_x_;
    double locked_ori_y_;
    double locked_ori_z_;
    double locked_ori_w_;
    bool   orientation_captured_;

    std::atomic<bool> joint_states_received_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MergedDetectionCartesianNode>());
    rclcpp::shutdown();
    return 0;
}