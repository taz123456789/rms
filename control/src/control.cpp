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

class ArmKinematicsNode : public rclcpp::Node
{
public:
    ArmKinematicsNode()
        : Node("arm_kinematics_node"),
        joint_names_{ "j1z", "j1x", "j2z", "j2x", "j3z", "j3x" },
        link_lengths_{ 0.3, 0.3, 0.2 },// links change
        ik_tolerance_(0.01),
        max_iterations_(100),
        joint_limit_z_min_(0.0),
        joint_limit_z_max_(2.0 * M_PI),//0° -> 360°
        joint_limit_x_min_(-M_PI / 3.0),//-60°
        joint_limit_x_max_(M_PI / 3.0)//60°
    {
        //get parameters
        this->declare_parameter("link_lengths", link_lengths_);
        this->declare_parameter("ik_tolerance", ik_tolerance_);
        this->declare_parameter("max_iterations", max_iterations_);
        this->get_parameter("link_lengths", link_lengths_);
        this->get_parameter("ik_tolerance", ik_tolerance_);
        this->get_parameter("max_iterations", max_iterations_);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);
        ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm/ee_pose", 10);
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm/joint_command", 10,std::bind(&ArmKinematicsNode::jointCommandCallback, this, std::placeholders::_1));
        ik_target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/arm/ik_target", 10,std::bind(&ArmKinematicsNode::ikTargetCallback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        //20 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArmKinematicsNode::timerCallback, this));
        //init
        joint_angles_ = Eigen::VectorXd::Zero(6);
        RCLCPP_INFO(this->get_logger(), "Arm Kinematics Node started");
        RCLCPP_INFO(this->get_logger(), "Joint limits: Z = [0, 360] deg, X = [-60, 60] deg");
    }

private:
    const std::vector<std::string> joint_names_;
    std::vector<double> link_lengths_;
    double ik_tolerance_;
    int max_iterations_;
    const double joint_limit_z_min_, joint_limit_z_max_;
    const double joint_limit_x_min_, joint_limit_x_max_;

    //current state
    Eigen::VectorXd joint_angles_;//z1, x1, z2, x2, z3, x3 rad
    Eigen::Vector3d ee_position_;
    Eigen::Quaterniond ee_orientation_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ik_target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6)
        {
            RCLCPP_WARN(this->get_logger(), "joint command has insufficient positions");
            return;
        }
        for (size_t i = 0; i < 6; ++i)
        {
            joint_angles_[i] = msg->position[i];
        }
        //fk
        clampAngles(joint_angles_);
        forwardKinematics();
        RCLCPP_DEBUG(this->get_logger(), "received joint command");
    }

    void ikTargetCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        Eigen::Vector3d target_pos(msg->position.x,
                                   msg->position.y,
                                   msg->position.z);
        RCLCPP_INFO(this->get_logger(), "ik target: (%.3f, %.3f, %.3f)",
            target_pos.x(), target_pos.y(), target_pos.z());
        // ik
        Eigen::VectorXd solution(6);
        bool success = inverseKinematics(target_pos, solution);
        if (success)
        {
            joint_angles_ = solution;
            forwardKinematics();// recomputing ee pose
            RCLCPP_INFO(this->get_logger(), "ik solved successfully");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ik failed");
        }
    }

    void timerCallback()
    {
        publishStates();
        publishTransforms();
    }

    void forwardKinematics()
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        //joint positions for TF
        std::vector<Eigen::Vector3d> joint_positions;
        joint_positions.push_back(pos);// base

        for (int i = 0; i < 3; ++i)
        {
            double z = joint_angles_[2 * i];    
            double x = joint_angles_[2 * i + 1]; 
            //about Z then X
            Eigen::Matrix3d Rz = Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            Eigen::Matrix3d Rx = Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix();
            R = R * Rz * Rx; 
            //vector along X axis
            Eigen::Vector3d link_vec = R * Eigen::Vector3d(link_lengths_[i], 0.0, 0.0);
            pos += link_vec;
            joint_positions.push_back(pos);  //next joint pos
        }

        //ee position & orientation
        ee_position_ = pos;
        ee_orientation_ = Eigen::Quaterniond(R);
    }

    Eigen::Vector3d computeEndEffectorPosition(const Eigen::VectorXd& angles)
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        for (int i = 0; i < 3; ++i)
        {
            double z = angles[2 * i];
            double x = angles[2 * i + 1];
            R = R * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix();
            pos += R * Eigen::Vector3d(link_lengths_[i], 0.0, 0.0);
        }
        return pos;
    }

    bool inverseKinematics(const Eigen::Vector3d& target_pos, Eigen::VectorXd& solution)
    {
        // current ang=init
        solution = joint_angles_;
        double error = std::numeric_limits<double>::max();
        int iter = 0;
        while (error > ik_tolerance_ && iter < max_iterations_)
        {
            //current ee position
            Eigen::Vector3d ee_pos = computeEndEffectorPosition(solution);
            error = (target_pos - ee_pos).norm();
            if (error <= ik_tolerance_)
                break;

            //from last joint -> base (joint2 -> joint1 -> joint0)
            for (int j = 2; j >= 0; --j)
            {
                Eigen::Vector3d joint_pos = computeJointPosition(solution, j);
                // vectors (from joint -> EE & target)
                Eigen::Vector3d to_ee = ee_pos - joint_pos;
                Eigen::Vector3d to_target = target_pos - joint_pos;
                double to_ee_norm = to_ee.norm();
                double to_target_norm = to_target.norm();
                if (to_ee_norm < 1e-6 || to_target_norm < 1e-6)
                    continue;

                to_ee.normalize();
                to_target.normalize();
                Eigen::Vector3d axis_world = to_ee.cross(to_target);
                double axis_norm = axis_world.norm();
                if (axis_norm < 1e-6)
                    continue;
                axis_world /= axis_norm;

                double angle = std::acos(to_ee.dot(to_target));
                angle = std::min(angle, 0.5);  //limit step size for stability

                //project this world rotation onto the joint's local axes.
                // current orientation of joint j
                Eigen::Matrix3d R_j = computeJointOrientation(solution, j);
                Eigen::Vector3d local_z = R_j * Eigen::Vector3d::UnitZ();
                Eigen::Vector3d local_x = R_j * Eigen::Vector3d::UnitX();
                //rotate first about local Z, then about local X.
                double dz = axis_world.dot(local_z) * angle;
                double dx = axis_world.dot(local_x) * angle;
                //1st->Z 2nd-> X)
                int idx_z = 2 * j;
                int idx_x = 2 * j + 1;
                solution[idx_z] += dz;
                solution[idx_x] += dx;
                applyJointLimits(solution[idx_z], idx_z);
                applyJointLimits(solution[idx_x], idx_x);
                ee_pos = computeEndEffectorPosition(solution);
                error = (target_pos - ee_pos).norm();
                if (error <= ik_tolerance_)
                    break;
            }
            ++iter;
        }

        if (error <= ik_tolerance_)
        {
            RCLCPP_DEBUG(this->get_logger(), "IK converged in %d iterations, error = %.4f", iter, error);
            return true;
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "IK failed after %d iterations, error = %.4f", iter, error);
            return false;
        }
    }

    //0 = base, 1 = after first link, 2 = after second
    Eigen::Vector3d computeJointPosition(const Eigen::VectorXd& angles, int joint_idx)
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        for (int i = 0; i < joint_idx; ++i)
        {
            double z = angles[2 * i];
            double x = angles[2 * i + 1];
            R = R * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix();
            pos += R * Eigen::Vector3d(link_lengths_[i], 0.0, 0.0);
        }
        return pos;
    }

    //orientation ofthe frame after joint j's rotations
    Eigen::Matrix3d computeJointOrientation(const Eigen::VectorXd& angles, int joint_idx)
    {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        for (int i = 0; i <= joint_idx; ++i)
        {
            double z = angles[2 * i];
            double x = angles[2 * i + 1];
            R = R * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix();
        }
        return R;
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
        if (idx % 2 == 0)
        {
            if (angle < joint_limit_z_min_) angle = joint_limit_z_min_;
            if (angle > joint_limit_z_max_) angle = joint_limit_z_max_;
        }
        else 
        {
            if (angle < joint_limit_x_min_) angle = joint_limit_x_min_;
            if (angle > joint_limit_x_max_) angle = joint_limit_x_max_;
        }
    }


    void publishStates()
    {
        // jointstate msg
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = joint_names_;
        joint_msg.position.resize(6);
        for (size_t i = 0; i < 6; ++i)
            joint_msg.position[i] = joint_angles_[i];
        joint_state_pub_->publish(joint_msg);
        // ee pose message
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

        //transforms from joint angles
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);

        //base -> joint1
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "base";
        t.child_frame_id = "joint1";
        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = 0;
        Eigen::Quaterniond q(R);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        for (int i = 0; i < 3; ++i)
        {
            double z = joint_angles_[2 * i];
            double x = joint_angles_[2 * i + 1];
            R = R * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                * Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Vector3d link_vec = R * Eigen::Vector3d(link_lengths_[i], 0.0, 0.0);
            pos += link_vec;

            //transform from previous joint -> current joint 
            if (i < 2)
            {
                geometry_msgs::msg::TransformStamped t_joint;
                t_joint.header.stamp = now;
                t_joint.header.frame_id = "joint" + std::to_string(i + 1);
                t_joint.child_frame_id = "joint" + std::to_string(i + 2);
                t_joint.transform.translation.x = link_vec.x();
                t_joint.transform.translation.y = link_vec.y();
                t_joint.transform.translation.z = link_vec.z();
                Eigen::Quaterniond q_link(R);
                t_joint.transform.rotation.x = q_link.x();
                t_joint.transform.rotation.y = q_link.y();
                t_joint.transform.rotation.z = q_link.z();
                t_joint.transform.rotation.w = q_link.w();
                tf_broadcaster_->sendTransform(t_joint);
            }
        }

        // joint3 to ee
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
    rclcpp::spin(std::make_shared<ArmKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}