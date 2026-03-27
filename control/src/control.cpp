#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <chrono>
#include <iomanip>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ArmKinematicsNode : public rclcpp::Node
{
public:
    ArmKinematicsNode()
        : Node("arm_kinematics_node"),
        joint_names_{ "j1z", "j1x", "j2z", "j2x", "j3z", "j3x" },
        link_lengths_{ 0.3, 0.3, 0.2 },
        ik_tolerance_(0.005),
        max_iterations_(1000),
        damping_factor_(0.01),
        joint_limit_z_min_(0.0),
        joint_limit_z_max_(2.0 * M_PI),
        joint_limit_x_min_(-M_PI * 0.45),
        joint_limit_x_max_(M_PI * 0.45)
    {
        // Get parameters
        this->declare_parameter("link_lengths", link_lengths_);
        this->declare_parameter("ik_tolerance", ik_tolerance_);
        this->declare_parameter("max_iterations", max_iterations_);
        this->declare_parameter("damping_factor", damping_factor_);
        this->get_parameter("link_lengths", link_lengths_);
        this->get_parameter("ik_tolerance", ik_tolerance_);
        this->get_parameter("max_iterations", max_iterations_);
        this->get_parameter("damping_factor", damping_factor_);

        // Publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);
        ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm/ee_pose", 10);
        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_command", 10);
        fk_show_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm/fkshow", 10);

        // Subscribers
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm/joint_command", 10,
            std::bind(&ArmKinematicsNode::jointCommandCallback, this, std::placeholders::_1));
        ik_target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/arm/ik_target", 10,
            std::bind(&ArmKinematicsNode::ikTargetCallback, this, std::placeholders::_1));
        fk_show_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/arm/fkshow", 10,
            std::bind(&ArmKinematicsNode::fkShowCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 20 Hz timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArmKinematicsNode::timerCallback, this));

        // Initialize joint angles to a reasonable starting position
        joint_angles_ = Eigen::VectorXd::Zero(6);
        joint_angles_[1] = M_PI / 6.0;  // j1x: 30 degrees
        joint_angles_[3] = M_PI / 6.0;  // j2x: 30 degrees  
        joint_angles_[5] = M_PI / 6.0;  // j3x: 30 degrees

        forwardKinematics();

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Arm Kinematics Node Started");
        RCLCPP_INFO(this->get_logger(), "IK Method: Jacobian-based with Damped Least Squares");
        RCLCPP_INFO(this->get_logger(), "Link lengths: [%.2f, %.2f, %.2f] m",
            link_lengths_[0], link_lengths_[1], link_lengths_[2]);
        RCLCPP_INFO(this->get_logger(), "Max reach: %.2f m",
            link_lengths_[0] + link_lengths_[1] + link_lengths_[2]);
        RCLCPP_INFO(this->get_logger(), "Initial EE position: (%.3f, %.3f, %.3f)",
            ee_position_.x(), ee_position_.y(), ee_position_.z());
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    const std::vector<std::string> joint_names_;
    std::vector<double> link_lengths_;
    double ik_tolerance_;
    int max_iterations_;
    double damping_factor_;
    const double joint_limit_z_min_, joint_limit_z_max_;
    const double joint_limit_x_min_, joint_limit_x_max_;

    Eigen::VectorXd joint_angles_;
    Eigen::Vector3d ee_position_;
    Eigen::Quaterniond ee_orientation_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr fk_show_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ik_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr fk_show_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    struct IKStats {
        int total_calls = 0;
        int successful_calls = 0;
        double avg_iterations = 0;
        double avg_time_ms = 0;
        double total_time_ms = 0;
        double min_time_ms = std::numeric_limits<double>::max();
        double max_time_ms = 0;
    } ik_stats_;

    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6)
        {
            RCLCPP_WARN(this->get_logger(), "Joint command has insufficient positions");
            return;
        }
        for (size_t i = 0; i < 6; ++i)
        {
            joint_angles_[i] = msg->position[i];
        }
        clampAngles(joint_angles_);
        forwardKinematics();
        RCLCPP_INFO(this->get_logger(), "Joint command received and applied");
    }

    void fkShowCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "FORWARD KINEMATICS DISPLAY");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Display current joint angles
        RCLCPP_INFO(this->get_logger(), "Current Joint Angles (radians):");
        RCLCPP_INFO(this->get_logger(), "  j1z: %.4f rad (%.1f deg)", joint_angles_[0], joint_angles_[0] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  j1x: %.4f rad (%.1f deg)", joint_angles_[1], joint_angles_[1] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  j2z: %.4f rad (%.1f deg)", joint_angles_[2], joint_angles_[2] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  j2x: %.4f rad (%.1f deg)", joint_angles_[3], joint_angles_[3] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  j3z: %.4f rad (%.1f deg)", joint_angles_[4], joint_angles_[4] * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  j3x: %.4f rad (%.1f deg)", joint_angles_[5], joint_angles_[5] * 180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "\nEnd-Effector Position:");
        RCLCPP_INFO(this->get_logger(), "  x: %.4f m", ee_position_.x());
        RCLCPP_INFO(this->get_logger(), "  y: %.4f m", ee_position_.y());
        RCLCPP_INFO(this->get_logger(), "  z: %.4f m", ee_position_.z());

        RCLCPP_INFO(this->get_logger(), "\nEnd-Effector Orientation (Quaternion):");
        RCLCPP_INFO(this->get_logger(), "  x: %.4f", ee_orientation_.x());
        RCLCPP_INFO(this->get_logger(), "  y: %.4f", ee_orientation_.y());
        RCLCPP_INFO(this->get_logger(), "  z: %.4f", ee_orientation_.z());
        RCLCPP_INFO(this->get_logger(), "  w: %.4f", ee_orientation_.w());

        RCLCPP_INFO(this->get_logger(), "\nDistance from base: %.4f m", ee_position_.norm());
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Also publish the current pose to the FK show topic
        auto response_msg = geometry_msgs::msg::Pose();
        response_msg.position.x = ee_position_.x();
        response_msg.position.y = ee_position_.y();
        response_msg.position.z = ee_position_.z();
        response_msg.orientation.x = ee_orientation_.x();
        response_msg.orientation.y = ee_orientation_.y();
        response_msg.orientation.z = ee_orientation_.z();
        response_msg.orientation.w = ee_orientation_.w();
        fk_show_pub_->publish(response_msg);
    }

    void ikTargetCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        Eigen::Vector3d target_pos(msg->position.x,
            msg->position.y,
            msg->position.z);

        // Optional orientation target (for future expansion)
        Eigen::Quaterniond target_orient(msg->orientation.w, msg->orientation.x,
            msg->orientation.y, msg->orientation.z);

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "INVERSE KINEMATICS SOLUTION");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Target Position: (%.3f, %.3f, %.3f)",
            target_pos.x(), target_pos.y(), target_pos.z());

        // Check reachability
        double max_reach = link_lengths_[0] + link_lengths_[1] + link_lengths_[2];
        double min_reach = std::abs(link_lengths_[0] - link_lengths_[1] - link_lengths_[2]);
        double distance = target_pos.norm();

        RCLCPP_INFO(this->get_logger(), "Distance from base: %.3f m (Reach: [%.3f, %.3f])",
            distance, min_reach, max_reach);

        if (distance > max_reach)
        {
            RCLCPP_WARN(this->get_logger(), "Target OUTSIDE workspace! Scaling to max reach...");
            target_pos = target_pos * (max_reach / distance);
            RCLCPP_INFO(this->get_logger(), "Adjusted target: (%.3f, %.3f, %.3f)",
                target_pos.x(), target_pos.y(), target_pos.z());
        }
        else if (distance < min_reach)
        {
            RCLCPP_WARN(this->get_logger(), "Target too close to base! Min reach: %.3f", min_reach);
        }

        Eigen::VectorXd solution(6);
        auto start_time = std::chrono::high_resolution_clock::now();
        bool success = inverseKinematicsJacobian(target_pos, solution);
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        updateStatistics(success, elapsed_ms);

        if (success)
        {
            Eigen::Vector3d achieved_pos = computeEndEffectorPosition(solution);
            double final_error = (target_pos - achieved_pos).norm();

            RCLCPP_INFO(this->get_logger(), "\n✓ IK SUCCESS!");
            RCLCPP_INFO(this->get_logger(), "  Computation Time: %.2f ms", elapsed_ms);
            RCLCPP_INFO(this->get_logger(), "  Iterations: %d", static_cast<int>(ik_stats_.avg_iterations));
            RCLCPP_INFO(this->get_logger(), "\nSolution Angles (radians):");
            RCLCPP_INFO(this->get_logger(), "  j1z: %.4f rad (%.1f deg)", solution[0], solution[0] * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "  j1x: %.4f rad (%.1f deg)", solution[1], solution[1] * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "  j2z: %.4f rad (%.1f deg)", solution[2], solution[2] * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "  j2x: %.4f rad (%.1f deg)", solution[3], solution[3] * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "  j3z: %.4f rad (%.1f deg)", solution[4], solution[4] * 180 / M_PI);
            RCLCPP_INFO(this->get_logger(), "  j3x: %.4f rad (%.1f deg)", solution[5], solution[5] * 180 / M_PI);

            RCLCPP_INFO(this->get_logger(), "\nAchieved Position: (%.3f, %.3f, %.3f)",
                achieved_pos.x(), achieved_pos.y(), achieved_pos.z());
            RCLCPP_INFO(this->get_logger(), "Final Position Error: %.4f m", final_error);
            RCLCPP_INFO(this->get_logger(), "\nPerformance Statistics:");
            RCLCPP_INFO(this->get_logger(), "  Success Rate: %.1f%% (%d/%d)",
                (ik_stats_.successful_calls * 100.0 / ik_stats_.total_calls),
                ik_stats_.successful_calls, ik_stats_.total_calls);
            RCLCPP_INFO(this->get_logger(), "  Avg Time: %.2f ms [%.2f-%.2f]",
                ik_stats_.avg_time_ms, ik_stats_.min_time_ms, ik_stats_.max_time_ms);

            joint_angles_ = solution;
            forwardKinematics();
            publishJointCommand();
        }
        else
        {
            Eigen::Vector3d final_pos = computeEndEffectorPosition(solution);
            double final_error = (target_pos - final_pos).norm();

            RCLCPP_ERROR(this->get_logger(), "\n✗ IK FAILED!");
            RCLCPP_ERROR(this->get_logger(), "  Iterations: %d", max_iterations_);
            RCLCPP_ERROR(this->get_logger(), "  Final Error: %.4f m", final_error);
            RCLCPP_ERROR(this->get_logger(), "  Target: (%.3f, %.3f, %.3f)",
                target_pos.x(), target_pos.y(), target_pos.z());
            RCLCPP_ERROR(this->get_logger(), "  Achieved: (%.3f, %.3f, %.3f)",
                final_pos.x(), final_pos.y(), final_pos.z());
        }
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    void updateStatistics(bool success, double elapsed_ms)
    {
        ik_stats_.total_calls++;
        if (success) ik_stats_.successful_calls++;
        ik_stats_.total_time_ms += elapsed_ms;
        ik_stats_.avg_time_ms = ik_stats_.total_time_ms / ik_stats_.total_calls;
        ik_stats_.min_time_ms = std::min(ik_stats_.min_time_ms, elapsed_ms);
        ik_stats_.max_time_ms = std::max(ik_stats_.max_time_ms, elapsed_ms);

        // Update average iterations (this is approximate)
        if (success && ik_stats_.avg_iterations == 0)
            ik_stats_.avg_iterations = max_iterations_ / 2;
    }

    void publishJointCommand()
    {
        auto cmd_msg = sensor_msgs::msg::JointState();
        cmd_msg.header.stamp = this->get_clock()->now();
        cmd_msg.name = joint_names_;
        cmd_msg.position.resize(6);
        for (size_t i = 0; i < 6; ++i)
        {
            cmd_msg.position[i] = joint_angles_[i];
        }
        joint_command_pub_->publish(cmd_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published joint command");
    }

    void timerCallback()
    {
        forwardKinematics();
        publishStates();
        publishTransforms();
    }

    void forwardKinematics()
    {
        Eigen::Matrix4d T = computeTransformMatrix(joint_angles_);
        ee_position_ = T.block<3, 1>(0, 3);
        ee_orientation_ = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    }

    Eigen::Matrix4d computeTransformMatrix(const Eigen::VectorXd& angles)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (int i = 0; i < 3; ++i)
        {
            double z_rot = angles[2 * i];
            double x_rot = angles[2 * i + 1];

            Eigen::Matrix4d T_z;
            T_z << cos(z_rot), -sin(z_rot), 0, 0,
                sin(z_rot), cos(z_rot), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

            Eigen::Matrix4d T_x;
            T_x << 1, 0, 0, 0,
                0, cos(x_rot), -sin(x_rot), 0,
                0, sin(x_rot), cos(x_rot), 0,
                0, 0, 0, 1;

            Eigen::Matrix4d T_link;
            T_link << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, link_lengths_[i],
                0, 0, 0, 1;

            T = T * T_z * T_x * T_link;
        }

        return T;
    }

    Eigen::Vector3d computeEndEffectorPosition(const Eigen::VectorXd& angles)
    {
        Eigen::Matrix4d T = computeTransformMatrix(angles);
        return T.block<3, 1>(0, 3);
    }

    bool inverseKinematicsJacobian(const Eigen::Vector3d& target_pos, Eigen::VectorXd& solution)
    {
        solution = joint_angles_;
        double error = std::numeric_limits<double>::max();
        int iter = 0;
        double damping = damping_factor_;
        double max_step = 0.15;

        RCLCPP_INFO(this->get_logger(), "\nJacobian IK Iterations:");
        RCLCPP_INFO(this->get_logger(), "iter | error (m) | joint updates");
        RCLCPP_INFO(this->get_logger(), "-----|-----------|--------------");

        while (error > ik_tolerance_ && iter < max_iterations_)
        {
            Eigen::Vector3d ee_pos = computeEndEffectorPosition(solution);
            Eigen::Vector3d error_vec = target_pos - ee_pos;
            error = error_vec.norm();

            if (iter % 50 == 0 || error < 0.1) {
                RCLCPP_INFO(this->get_logger(), "%4d | %9.4f | [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    iter, error,
                    solution[0], solution[1], solution[2], solution[3], solution[4], solution[5]);
            }

            if (error <= ik_tolerance_)
                break;

            // Compute Jacobian matrix using proper method
            Eigen::MatrixXd J(3, 6);
            computeJacobian(solution, J);

            // Display Jacobian for debugging (first few iterations)
            if (iter < 3) {
                RCLCPP_DEBUG(this->get_logger(), "Jacobian Matrix (iter %d):", iter);
                for (int i = 0; i < 3; ++i) {
                    RCLCPP_DEBUG(this->get_logger(), "  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                        J(i, 0), J(i, 1), J(i, 2), J(i, 3), J(i, 4), J(i, 5));
                }
            }

            // Damped Least Squares solution: dq = J^T * (J*J^T + λ²I)⁻¹ * error
            Eigen::MatrixXd JJt = J * J.transpose();
            Eigen::MatrixXd damped = JJt + damping * damping * Eigen::MatrixXd::Identity(3, 3);

            // Solve using LDLT for better stability
            Eigen::VectorXd dq = J.transpose() * damped.ldlt().solve(error_vec);

            // Apply joint angle changes with step limiting
            for (int i = 0; i < 6; ++i)
            {
                if (std::abs(dq[i]) > max_step)
                    dq[i] = (dq[i] > 0 ? max_step : -max_step);

                solution[i] += dq[i];
                applyJointLimits(solution[i], i);
            }

            // Adaptive damping based on error
            if (error < 0.05)
                damping = damping_factor_ * 0.1;
            else if (error < 0.1)
                damping = damping_factor_ * 0.3;
            else if (error < 0.3)
                damping = damping_factor_ * 0.6;
            else
                damping = damping_factor_;

            iter++;

            // Early exit if stuck
            if (iter > 100 && error < 0.2 && error > 0.15) {
                RCLCPP_WARN(this->get_logger(), "Stuck at error %.4f, applying random perturbation", error);
                for (int i = 0; i < 6; ++i) {
                    solution[i] += (rand() % 100 - 50) * 0.005;
                    applyJointLimits(solution[i], i);
                }
            }
        }

        ik_stats_.avg_iterations = (ik_stats_.avg_iterations * ik_stats_.total_calls + iter) / (ik_stats_.total_calls + 1);

        if (iter >= max_iterations_) {
            RCLCPP_WARN(this->get_logger(), "Max iterations reached without convergence");
            return false;
        }

        return error <= ik_tolerance_;
    }

    void computeJacobian(const Eigen::VectorXd& angles, Eigen::MatrixXd& J)
    {
        // Compute all joint positions and axes
        std::vector<Eigen::Vector3d> joint_positions(3);
        std::vector<Eigen::Vector3d> joint_axes(6);

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);

        // Calculate current end-effector position
        Eigen::Vector3d ee_pos = computeEndEffectorPosition(angles);

        for (int i = 0; i < 3; ++i)
        {
            double z_rot = angles[2 * i];
            double x_rot = angles[2 * i + 1];

            // Store joint position before rotation
            joint_positions[i] = pos;

            // Z rotation axis (in current frame)
            joint_axes[2 * i] = R * Eigen::Vector3d::UnitZ();

            // Apply Z rotation
            Eigen::Matrix3d Rz = Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            R = R * Rz;

            // X rotation axis (in current frame after Z rotation)
            joint_axes[2 * i + 1] = R * Eigen::Vector3d::UnitX();

            // Apply X rotation
            Eigen::Matrix3d Rx = Eigen::AngleAxisd(x_rot, Eigen::Vector3d::UnitX()).toRotationMatrix();
            R = R * Rx;

            // Add link translation
            pos += R * Eigen::Vector3d(0.0, 0.0, link_lengths_[i]);
        }

        // Compute Jacobian columns using the cross product method
        // For revolute joints: J_i = axis_i × (ee_pos - joint_pos_i)
        for (int i = 0; i < 6; ++i)
        {
            int joint_idx = i / 2;
            Eigen::Vector3d r = ee_pos - joint_positions[joint_idx];
            // J.col(i) = joint_axes[i].cross(r);
            J.block<3, 1>(0, i) = joint_axes[i].cross(r);
            J.block<3, 1>(3, i) = joint_axes[i];
            //// Debug output for first few iterations
            //if (ik_stats_.total_calls < 3) {
            //    RCLCPP_DEBUG(this->get_logger(), "Joint %d axis: [%.3f, %.3f, %.3f], r: [%.3f, %.3f, %.3f], J: [%.3f, %.3f, %.3f]",
            //        i, joint_axes[i].x(), joint_axes[i].y(), joint_axes[i].z(),
            //        r.x(), r.y(), r.z(),
            //        J(0, i), J(1, i), J(2, i));
        }
    }
}

void clampAngles(Eigen::VectorXd& angles)
{
    for (int i = 0; i < 6; ++i)
    {
        applyJointLimits(angles[i], i);
    }
}

void applyJointLimits(double& angle, int idx)
{
    if (idx % 2 == 0)  // Z rotation
    {
        // Normalize angle to [0, 2π]
        angle = std::fmod(angle, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;

        if (angle < joint_limit_z_min_) angle = joint_limit_z_min_;
        if (angle > joint_limit_z_max_) angle = joint_limit_z_max_;
    }
    else  // X rotation
    {
        if (angle < joint_limit_x_min_) angle = joint_limit_x_min_;
        if (angle > joint_limit_x_max_) angle = joint_limit_x_max_;
    }
}

void publishStates()
{
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = joint_names_;
    joint_msg.position.resize(6);
    for (size_t i = 0; i < 6; ++i)
        joint_msg.position[i] = joint_angles_[i];
    joint_state_pub_->publish(joint_msg);

    auto pose_msg = geometry_msgs::msg::Pose();
    pose_msg.position.x = ee_position_.x();
    pose_msg.position.y = ee_position_.y();
    pose_msg.position.z = ee_position_.z();
    pose_msg.orientation.x = ee_orientation_.x();
    pose_msg.orientation.y = ee_orientation_.y();
    pose_msg.orientation.z = ee_orientation_.z();
    pose_msg.orientation.w = ee_orientation_.w();
    ee_pose_pub_->publish(pose_msg);
}

void publishTransforms()
{
    builtin_interfaces::msg::Time now = this->get_clock()->now();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos(0.0, 0.0, 0.0);

    geometry_msgs::msg::TransformStamped t_base;
    t_base.header.stamp = now;
    t_base.header.frame_id = "base";
    t_base.child_frame_id = "joint1";
    t_base.transform.translation.x = 0;
    t_base.transform.translation.y = 0;
    t_base.transform.translation.z = 0;
    t_base.transform.rotation.x = 0;
    t_base.transform.rotation.y = 0;
    t_base.transform.rotation.z = 0;
    t_base.transform.rotation.w = 1;
    tf_broadcaster_->sendTransform(t_base);

    for (int i = 0; i < 3; ++i)
    {
        double z_rot = joint_angles_[2 * i];
        double x_rot = joint_angles_[2 * i + 1];

        R = R * Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()).toRotationMatrix()
            * Eigen::AngleAxisd(x_rot, Eigen::Vector3d::UnitX()).toRotationMatrix();

        Eigen::Vector3d link_vec = R * Eigen::Vector3d(0.0, 0.0, link_lengths_[i]);
        pos += link_vec;

        if (i < 2)
        {
            geometry_msgs::msg::TransformStamped t_joint;
            t_joint.header.stamp = now;
            t_joint.header.frame_id = "joint" + std::to_string(i + 1);
            t_joint.child_frame_id = "joint" + std::to_string(i + 2);
            t_joint.transform.translation.x = link_vec.x();
            t_joint.transform.translation.y = link_vec.y();
            t_joint.transform.translation.z = link_vec.z();

            Eigen::Quaterniond q(R);
            t_joint.transform.rotation.x = q.x();
            t_joint.transform.rotation.y = q.y();
            t_joint.transform.rotation.z = q.z();
            t_joint.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t_joint);
        }
    }

    geometry_msgs::msg::TransformStamped t_ee;
    t_ee.header.stamp = now;
    t_ee.header.frame_id = "joint3";
    t_ee.child_frame_id = "end_effector";
    t_ee.transform.translation.x = 0;
    t_ee.transform.translation.y = 0;
    t_ee.transform.translation.z = 0;
    t_ee.transform.rotation.x = 0;
    t_ee.transform.rotation.y = 0;
    t_ee.transform.rotation.z = 0;
    t_ee.transform.rotation.w = 1;
    tf_broadcaster_->sendTransform(t_ee);
}
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmKinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}