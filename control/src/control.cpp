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

// Custom message for trajectory command (you need to define this .msg)
// For simplicity, we'll reuse JointState which can carry position, velocity, acceleration
#include <sensor_msgs/msg/joint_state.hpp>

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
        // Declare and get parameters (existing)
        this->declare_parameter("link_lengths", link_lengths_);
        this->declare_parameter("ik_tolerance", ik_tolerance_);
        this->declare_parameter("max_iterations", max_iterations_);
        this->declare_parameter("damping_factor", damping_factor_);
        this->get_parameter("link_lengths", link_lengths_);
        this->get_parameter("ik_tolerance", ik_tolerance_);
        this->get_parameter("max_iterations", max_iterations_);
        this->get_parameter("damping_factor", damping_factor_);

        // === RNEA Parameters ===
        this->declare_parameter("link_masses", std::vector<double>{1.0, 0.8, 0.6});
        this->declare_parameter("gravity", 9.81);
        this->get_parameter("link_masses", link_masses_);
        this->get_parameter("gravity", gravity_);
        // Inertias approximated as slender rods (I = 1/12 * m * L^2 about COM)
        setupLinkDynamics();

        // Publishers (existing)
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);
        ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm/ee_pose", 10);
        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_command", 10);
        fk_show_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm/fkshow", 10);
        // New publisher for torques
        torque_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_torques", 10);

        // Subscribers (existing)
        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm/joint_command", 10,
            std::bind(&ArmKinematicsNode::jointCommandCallback, this, std::placeholders::_1));
        ik_target_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/arm/ik_target", 10,
            std::bind(&ArmKinematicsNode::ikTargetCallback, this, std::placeholders::_1));
        fk_show_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/arm/fkshow", 10,
            std::bind(&ArmKinematicsNode::fkShowCallback, this, std::placeholders::_1));
        // New subscriber for trajectory command (joint positions, velocities, accelerations)
        traj_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/arm/desired_trajectory", 10,
            std::bind(&ArmKinematicsNode::trajectoryCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 20 Hz timer (existing)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArmKinematicsNode::timerCallback, this));

        // Initialize joint angles (existing)
        joint_angles_ = Eigen::VectorXd::Zero(6);
        joint_angles_[1] = M_PI / 6.0;
        joint_angles_[3] = M_PI / 6.0;
        joint_angles_[5] = M_PI / 6.0;

        forwardKinematics();

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Arm Kinematics Node Started with RNEA");
        RCLCPP_INFO(this->get_logger(), "Link masses: [%.2f, %.2f, %.2f] kg",
            link_masses_[0], link_masses_[1], link_masses_[2]);
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

private:
    // Existing members
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

    // RNEA data
    std::vector<double> link_masses_;               // masses of each link (3 links)
    double gravity_;                                 // gravity magnitude (positive)
    struct LinkDynamic {
        double mass;
        Eigen::Vector3d com;           // center of mass in link frame (usually along Z)
        Eigen::Matrix3d inertia;       // inertia tensor about COM (3x3)
        Eigen::Vector3d axis;          // joint axis (for revolute joints)
        Eigen::Vector3d translation;   // translation from joint i to joint i+1 (in link frame)
    };
    std::vector<LinkDynamic> links_;    // size 6 (one per joint, though some share same physical link)
    // We'll map each joint to a virtual "link" (the part after that joint)
    // For simplicity, we treat each joint as a separate body with its own mass, COM, inertia.

    // Publishers & subscribers (existing + new)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ee_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr fk_show_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_pub_;      // new

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ik_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr fk_show_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr traj_sub_;     // new

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

    // ----------------------------------------------------------------------
    // RNEA Implementation (explicit 3D vectors, matching your style)
    // ----------------------------------------------------------------------
    void setupLinkDynamics()
    {
        // We have 6 joints, but only 3 physical links. We'll assign each joint's
        // "link" as the portion from that joint to the next joint.
        // For simplicity, we give each joint a mass equal to the corresponding link mass
        // (i.e., joint 0 and 1 share link 1 mass, but that would double count; better to split).
        // A more accurate approach: each link (after the joint) has its own mass.
        // We'll treat each joint as a separate body with appropriate mass distribution.
        // Here we approximate: the mass of link i is distributed between its two joints.
        // For simplicity, we'll give each revolute joint its own mass (half of link mass).
        // For better accuracy, you would define 3 rigid bodies (links) and compute their dynamics.
        // We'll keep it simple for demonstration.

        links_.resize(6);
        // Joint 0 (Z) and Joint 1 (X) belong to first physical link.
        // We'll assign masses proportionally to the part after each joint.
        double m_link1 = link_masses_[0];
        double m_link2 = link_masses_[1];
        double m_link3 = link_masses_[2];

        // Link 1 length = link_lengths_[0]
        // Assume uniform rod: COM at half length, inertia = 1/12 * m * L^2 about COM.
        double L1 = link_lengths_[0];
        double L2 = link_lengths_[1];
        double L3 = link_lengths_[2];

        // For joint 0 (Z rotation) – the mass from the base to joint 0 is negligible? Actually,
        // we need the mass of the part that rotates with joint 0. For simplicity, we'll assign
        // a small mass to joint 0 and the rest to joint 1. But it's better to treat the whole
        // first link as a single body after joint 0 and before joint 2? This is messy.
        // For a clean RNEA, each joint should have its own body. So we'll create 6 bodies,
        // each with its own mass, COM, and inertia. This may be an oversimplification, but
        // it works for demonstration. You can refine later.

        // For simplicity, we'll just assign each joint a fraction of the link's mass.
        // This is not physically accurate but will illustrate the algorithm.
        // In a real application, you would compute the mass properties of each rigid body
        // that moves with each joint.

        // Better: Create 3 bodies, each with a single joint (spherical). But we have 6 joints.
        // Actually, the arm has 3 spherical joints. Each spherical joint has 2 DOF but is physically one body.
        // So we should have 3 bodies, each with a single spherical joint (2 revolute axes at the same point).
        // The RNEA can handle that by giving each body a 6x6 spatial inertia and a 6x2 motion subspace.
        // That would be more correct. But for simplicity in this code, we'll keep the 6-joint model
        // and assign each joint a small mass that sums to the total link mass.

        // Let's distribute the link mass evenly between its two joints.
        double m_j0 = m_link1 * 0.5;
        double m_j1 = m_link1 * 0.5;
        double m_j2 = m_link2 * 0.5;
        double m_j3 = m_link2 * 0.5;
        double m_j4 = m_link3 * 0.5;
        double m_j5 = m_link3 * 0.5;

        // COM of each "link" (the part after the joint) – assume uniform rod from joint to next joint.
        // For joint i, the COM is at half the translation length from that joint to the next.
        // For joint 0, translation is along Z: r = [0,0,L1]; COM at L1/2 from joint 0.
        // For joint 1, translation is also along Z (since the link extends along Z after X rotation).
        // Actually, after joint 1 (X rotation), the link extends along the new Z axis (which is the rotated X).
        // So the translation vector for joint 1 is also [0,0,L1] (in its own frame). Same for others.
        // So we can set COM at [0,0, L/2] in the link frame.

        // Inertia about COM: for a slender rod about its COM, I = (1/12)*m*L^2 for axis perpendicular,
        // and 0 for axis along the rod. We'll approximate as diagonal.
        double I_xx = (1.0 / 12.0) * m_j0 * L1 * L1;
        double I_yy = I_xx;
        double I_zz = 0.0;   // along rod

        // Set each joint's dynamic parameters
        // Joint 0 (Z)
        links_[0].mass = m_j0;
        links_[0].com = Eigen::Vector3d(0, 0, L1 / 2);
        links_[0].inertia = Eigen::Vector3d(I_xx, I_yy, I_zz).asDiagonal();
        links_[0].axis = Eigen::Vector3d::UnitZ();
        links_[0].translation = Eigen::Vector3d(0, 0, L1);  // to next joint (joint 1)
        // Joint 1 (X)
        links_[1].mass = m_j1;
        links_[1].com = Eigen::Vector3d(0, 0, L1 / 2);
        links_[1].inertia = Eigen::Vector3d(I_xx, I_yy, I_zz).asDiagonal();
        links_[1].axis = Eigen::Vector3d::UnitX();
        links_[1].translation = Eigen::Vector3d(0, 0, L1);  // to joint 2 (but joint 2 is at the same point? Actually after joint 1, the next joint (2) is at the end of link 1)
        // Joint 2 (Z) – second link
        I_xx = (1.0 / 12.0) * m_j2 * L2 * L2;
        links_[2].mass = m_j2;
        links_[2].com = Eigen::Vector3d(0, 0, L2 / 2);
        links_[2].inertia = Eigen::Vector3d(I_xx, I_xx, 0).asDiagonal();
        links_[2].axis = Eigen::Vector3d::UnitZ();
        links_[2].translation = Eigen::Vector3d(0, 0, L2);
        // Joint 3 (X)
        links_[3].mass = m_j3;
        links_[3].com = Eigen::Vector3d(0, 0, L2 / 2);
        links_[3].inertia = Eigen::Vector3d(I_xx, I_xx, 0).asDiagonal();
        links_[3].axis = Eigen::Vector3d::UnitX();
        links_[3].translation = Eigen::Vector3d(0, 0, L2);
        // Joint 4 (Z) – third link
        I_xx = (1.0 / 12.0) * m_j4 * L3 * L3;
        links_[4].mass = m_j4;
        links_[4].com = Eigen::Vector3d(0, 0, L3 / 2);
        links_[4].inertia = Eigen::Vector3d(I_xx, I_xx, 0).asDiagonal();
        links_[4].axis = Eigen::Vector3d::UnitZ();
        links_[4].translation = Eigen::Vector3d(0, 0, L3);
        // Joint 5 (X)
        links_[5].mass = m_j5;
        links_[5].com = Eigen::Vector3d(0, 0, L3 / 2);
        links_[5].inertia = Eigen::Vector3d(I_xx, I_xx, 0).asDiagonal();
        links_[5].axis = Eigen::Vector3d::UnitX();
        links_[5].translation = Eigen::Vector3d(0, 0, L3); // no next joint, but used in backward pass
    }

    // Forward pass: compute ω, α, a_origin, a_com for each joint
    void forwardPass(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd,
        std::vector<Eigen::Vector3d>& omega,
        std::vector<Eigen::Vector3d>& alpha,
        std::vector<Eigen::Vector3d>& a_origin,
        std::vector<Eigen::Vector3d>& a_com)
    {
        int n = 6;
        omega.resize(n);
        alpha.resize(n);
        a_origin.resize(n);
        a_com.resize(n);

        // Initial conditions (base)
        Eigen::Vector3d omega_prev = Eigen::Vector3d::Zero();
        Eigen::Vector3d alpha_prev = Eigen::Vector3d::Zero();
        Eigen::Vector3d a_origin_prev = Eigen::Vector3d(0, 0, -gravity_);  // gravitational acceleration

        Eigen::Vector3d r_prev = Eigen::Vector3d::Zero();

        // Current orientation (to propagate axes correctly) – we need R for each link's orientation
        // We'll compute rotation matrices as we go, similar to computeJacobian.
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

        for (int i = 0; i < n; ++i)
        {
            const auto& link = links_[i];

            // Get joint axis in world frame (using current orientation)
            Eigen::Vector3d z = R * link.axis;

            // Angular velocity
            omega[i] = omega_prev + z * qd[i];

            // Angular acceleration
            alpha[i] = alpha_prev + z * qdd[i] + omega_prev.cross(z * qd[i]);

            // Linear acceleration of joint origin (before this joint's motion)
            // Use previous joint's origin and previous link's translation
            // (r_prev is translation from previous joint to current joint)
            a_origin[i] = a_origin_prev + alpha_prev.cross(r_prev) + omega_prev.cross(omega_prev.cross(r_prev));

            // Acceleration of center of mass
            a_com[i] = a_origin[i] + alpha[i].cross(link.com) + omega[i].cross(omega[i].cross(link.com));

            // Update for next iteration
            omega_prev = omega[i];
            alpha_prev = alpha[i];
            a_origin_prev = a_origin[i];
            r_prev = link.translation;

            // Update orientation for next joint: apply current joint rotation
            // For revolute joint, we need to rotate R by the joint's rotation.
            // But careful: the rotation axis is in current frame (z). We must apply the rotation.
            // The rotation amount is q[i]. However, we also need to account for the joint type.
            // For simplicity, we can update R using the same sequence as forward kinematics.
            // We'll recompute R from scratch? That would be inefficient. Better to update incrementally.
            // We'll compute the rotation due to this joint in the current orientation.
            // Since we have axis z in world frame, we can compute the rotation matrix for angle q[i] around axis z.
            Eigen::AngleAxisd rot_z(q[i], link.axis); // axis in link frame, but we need in world frame? Actually we need to rotate the current R.
            // The rotation is applied in the link frame, so we must multiply R by R_local (the rotation of the joint).
            // But we have the axis in link frame, so the local rotation matrix is AngleAxisd(q[i], link.axis).
            // We then update R = R * R_local.
            Eigen::Matrix3d R_local = Eigen::AngleAxisd(q[i], link.axis).toRotationMatrix();
            R = R * R_local;
        }
    }

    // Backward pass: compute joint forces and torques given external wrench at end-effector
    Eigen::VectorXd backwardPass(const std::vector<Eigen::Vector3d>& omega,
        const std::vector<Eigen::Vector3d>& alpha,
        const std::vector<Eigen::Vector3d>& a_com, const Eigen::VectorXd& q,
        const Eigen::VectorXd& external_wrench) // [Fx,Fy,Fz,Mx,My,Mz] at EE
    {
        int n = 6;
        std::vector<Eigen::Vector3d> f(n + 1); // force at joint (in world frame)
        std::vector<Eigen::Vector3d> m(n + 1); // moment at joint (in world frame)

        // End-effector wrench
        f[n] = external_wrench.head<3>();
        m[n] = external_wrench.tail<3>();

        Eigen::VectorXd tau(n);

        // We need to propagate forces backward. We also need the link orientations to transform
        // forces and moments. We'll recompute the orientations backward? For simplicity, we'll use
        // the same incremental orientation update as in forward pass, but we'll need to store them.
        // Instead, we can compute rotation matrices for each link during forward pass and store them.
        // Let's modify forwardPass to return orientations.

        // Since we didn't store orientations, we'll recompute them from q. That's acceptable for
        // clarity. In a performance-critical implementation, you'd store them.

        // We'll compute orientations for each link (world frame) using forward kinematics.
        std::vector<Eigen::Matrix3d> R_link(n);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        for (int i = 0; i < n; ++i)
        {
            const auto& link = links_[i];
            Eigen::Matrix3d R_local = Eigen::AngleAxisd(q[i], link.axis).toRotationMatrix();
            R = R * R_local;
            R_link[i] = R;
            // Also account for translation? No, orientation only.
        }

        // Now backward pass
        for (int i = n - 1; i >= 0; --i)
        {
            const auto& link = links_[i];
            Eigen::Vector3d r = link.translation;    // from joint i to joint i+1 (in link i frame)
            Eigen::Vector3d r_c = link.com;          // COM relative to joint i (in link i frame)

            // Transform force and moment from next joint (i+1) to current joint's frame
            // The next joint's force f[i+1] and moment m[i+1] are expressed in world frame?
            // Actually, in the explicit formulation, we keep everything in world frame to avoid
            // transformations. The backward pass equations we derived earlier used world frame
            // but required transforming f[i+1] to the current link's frame? Let's revisit.

            // In the standard explicit 3D formulation, we usually keep vectors in the same frame
            // (e.g., base frame) to avoid transformations. However, the equations become:
            // f[i] = f[i+1] + m_i * a_com[i]
            // m[i] = m[i+1] + r × f[i+1] + r_c × (m_i * a_com[i]) + I_i * α_i + ω_i × (I_i * ω_i)
            // This is valid if all vectors are expressed in the same frame (world frame). That's fine.
            // But the vectors r and r_c must be expressed in world frame. We have them in link frame.
            // We need to rotate them to world frame using the orientation of link i.
            Eigen::Vector3d r_world = R_link[i] * r;
            Eigen::Vector3d r_c_world = R_link[i] * r_c;

            // Inertial force at COM (in world frame)
            Eigen::Vector3d F_inertial = link.mass * a_com[i];

            // Inertial moment at COM (in world frame) – note that I_i must be rotated to world frame
            // The inertia tensor I_i is given in link frame; we need to express it in world frame.
            Eigen::Matrix3d I_world = R_link[i] * link.inertia * R_link[i].transpose();
            Eigen::Vector3d M_inertial = I_world * alpha[i] + omega[i].cross(I_world * omega[i]);

            // Force balance
            f[i] = f[i + 1] + F_inertial;

            // Moment balance
            m[i] = m[i + 1] + r_world.cross(f[i + 1]) + r_c_world.cross(F_inertial) + M_inertial;

            // Joint torque
            // The joint axis in world frame is R_link[i] * link.axis (but we can also get from omega calculation)
            // Actually, we have omega[i] = previous omega + z * qd, but we need z in world frame.
            Eigen::Vector3d z_world = R_link[i] * link.axis;
            tau[i] = m[i].dot(z_world);
        }

        return tau;
    }

    // Main RNEA function
    Eigen::VectorXd computeRNEA(const Eigen::VectorXd& q,
        const Eigen::VectorXd& qd,
        const Eigen::VectorXd& qdd,
        const Eigen::VectorXd& external_wrench)
    {
        std::vector<Eigen::Vector3d> omega, alpha, a_origin, a_com;
        forwardPass(q, qd, qdd, omega, alpha, a_origin, a_com);
        return backwardPass(omega, alpha, a_com, q, external_wrench);
    }

    // ----------------------------------------------------------------------
    // Existing methods (modified where necessary)
    // ----------------------------------------------------------------------
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
        // ... unchanged ...
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "FORWARD KINEMATICS DISPLAY");
        RCLCPP_INFO(this->get_logger(), "========================================");

        RCLCPP_INFO(this->get_logger(), "Current Joint Angles (radians):");
        for (int i = 0; i < 6; ++i)
            RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad (%.1f deg)",
                joint_names_[i].c_str(), joint_angles_[i], joint_angles_[i] * 180.0 / M_PI);

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
        // ... unchanged (same as original) ...
        Eigen::Vector3d target_pos(msg->position.x,
            msg->position.y,
            msg->position.z);

        Eigen::Quaterniond target_orient(msg->orientation.w, msg->orientation.x,
            msg->orientation.y, msg->orientation.z);

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "INVERSE KINEMATICS SOLUTION");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Target Position: (%.3f, %.3f, %.3f)",
            target_pos.x(), target_pos.y(), target_pos.z());

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
            for (int i = 0; i < 6; ++i)
                RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad (%.1f deg)",
                    joint_names_[i].c_str(), solution[i], solution[i] * 180 / M_PI);

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
            cmd_msg.position[i] = joint_angles_[i];
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

            // Compute Jacobian matrix (3x6)
            Eigen::MatrixXd J(3, 6);
            computeJacobian(solution, J);

            if (iter < 3) {
                RCLCPP_DEBUG(this->get_logger(), "Jacobian Matrix (iter %d):", iter);
                for (int i = 0; i < 3; ++i) {
                    RCLCPP_DEBUG(this->get_logger(), "  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                        J(i, 0), J(i, 1), J(i, 2), J(i, 3), J(i, 4), J(i, 5));
                }
            }

            // Damped Least Squares
            Eigen::MatrixXd JJt = J * J.transpose();
            Eigen::MatrixXd damped = JJt + damping * damping * Eigen::MatrixXd::Identity(3, 3);
            Eigen::VectorXd dq = J.transpose() * damped.ldlt().solve(error_vec);

            for (int i = 0; i < 6; ++i)
            {
                if (std::abs(dq[i]) > max_step)
                    dq[i] = (dq[i] > 0 ? max_step : -max_step);
                solution[i] += dq[i];
                applyJointLimits(solution[i], i);
            }

            // Adaptive damping
            if (error < 0.05)
                damping = damping_factor_ * 0.1;
            else if (error < 0.1)
                damping = damping_factor_ * 0.3;
            else if (error < 0.3)
                damping = damping_factor_ * 0.6;
            else
                damping = damping_factor_;

            iter++;

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

    // Fixed Jacobian: only linear part (3x6)
    void computeJacobian(const Eigen::VectorXd& angles, Eigen::MatrixXd& J)
    {
        std::vector<Eigen::Vector3d> joint_positions(3);
        std::vector<Eigen::Vector3d> joint_axes(6);

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        Eigen::Vector3d ee_pos = computeEndEffectorPosition(angles);

        for (int i = 0; i < 3; ++i)
        {
            double z_rot = angles[2 * i];
            double x_rot = angles[2 * i + 1];

            joint_positions[i] = pos;

            joint_axes[2 * i] = R * Eigen::Vector3d::UnitZ();

            Eigen::Matrix3d Rz = Eigen::AngleAxisd(z_rot, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            R = R * Rz;

            joint_axes[2 * i + 1] = R * Eigen::Vector3d::UnitX();

            Eigen::Matrix3d Rx = Eigen::AngleAxisd(x_rot, Eigen::Vector3d::UnitX()).toRotationMatrix();
            R = R * Rx;

            pos += R * Eigen::Vector3d(0.0, 0.0, link_lengths_[i]);
        }

        for (int i = 0; i < 6; ++i)
        {
            int joint_idx = i / 2;
            Eigen::Vector3d r = ee_pos - joint_positions[joint_idx];
            J.col(i) = joint_axes[i].cross(r);   // linear velocity part only
        }
    }

    void clampAngles(Eigen::VectorXd& angles)
    {
        for (int i = 0; i < 6; ++i)
            applyJointLimits(angles[i], i);
    }

    void applyJointLimits(double& angle, int idx)
    {
        if (idx % 2 == 0)  // Z rotation
        {
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

    // New callback for trajectory commands (RNEA demo)
    void trajectoryCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6 || msg->velocity.size() < 6 || msg->effort.size() < 6)
        {
            RCLCPP_WARN(this->get_logger(), "Trajectory message missing required arrays");
            return;
        }

        Eigen::VectorXd q(6), qd(6), qdd(6);
        for (int i = 0; i < 6; ++i)
        {
            q[i] = msg->position[i];
            qd[i] = msg->velocity[i];
            qdd[i] = msg->effort[i];  // we repurpose effort for acceleration
        }

        // External wrench (zero for now; can be read from another topic)
        Eigen::VectorXd external_wrench = Eigen::VectorXd::Zero(6);

        // Compute torques using RNEA
        Eigen::VectorXd tau = computeRNEA(q, qd, qdd, external_wrench);

        // Publish torques
        auto torque_msg = sensor_msgs::msg::JointState();
        torque_msg.header.stamp = this->get_clock()->now();
        torque_msg.name = joint_names_;
        torque_msg.effort.resize(6);
        for (int i = 0; i < 6; ++i)
            torque_msg.effort[i] = tau[i];
        torque_pub_->publish(torque_msg);

        RCLCPP_INFO(this->get_logger(), "Published computed torques: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            tau[0], tau[1], tau[2], tau[3], tau[4], tau[5]);
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
