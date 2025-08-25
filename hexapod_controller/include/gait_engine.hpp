#pragma once
#include <vector>
#include <array>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <optional>

struct Vec3 {
  double x, y, z;

  Vec3 operator+ (const Vec3& other) const {
    return {x + other.x, y + other.y, z + other.z};
  }

  Vec3() : x(0), y(0), z(0) {}
  Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct LegTransformMatrix {
    Eigen::Isometry3d body_to_coxa;
};

class GaitEngine {
public:
    GaitEngine(int cycle_duration);

    /// @brief Computes one point of the given leg's trajectory, if the computed point is unreachable, returns std::nullopt
    /// @param leg_id The ID of the leg (0-5)
    /// @param time The time at which to compute the IK
    /// @return The joint angles for the leg
    std::optional<std::vector<double>> getLegTrajectoryPoint(int leg_id, double time);

    /// @brief Computes one point of the given leg's trajectory, if the computed point is unreachable, returns std::nullopt
    /// @param leg_id The ID of the leg (0-5)
    /// @param time The time at which to compute the IK
    /// @param foot_pos_global Position of the foot in body frame, pass this argument for visualization purposes 
    /// @return The joint angles for the leg
    std::optional<std::vector<double>> getLegTrajectoryPoint(int leg_id, double time, Vec3& foot_pos_global);

    /// @brief Computes the forward kinematics for a leg
    /// @param joint_angles The joint angles for the leg
    /// @return The position of the end effector
    Vec3 computeLegFK(const std::array<double, 3>& joint_angles);

    /// @brief Initialize with leg frame transforms relative to body
    /// @param leg_id The ID of the leg (0-5)
    /// @param body_to_coxa Transform from body frame to coxa frame 
    void setLegFrames(int leg_id,const Eigen::Isometry3d& body_to_coxa);

    /// @brief Get leg endpoint
    /// IMPORTANT: returned position is local to coxa frame!!!
    /// @param leg_id The ID of the leg (0-5)
    /// @return The position of the leg endpoint in the body frame
    Eigen::Vector3d getLegEndpoint() {
        auto endpoint_pos = computeLegFK({0, 0, 0});
        return {endpoint_pos.x, endpoint_pos.y, endpoint_pos.z};
    }

  private:
    const struct {
        double d1 = 0; // link offset for joint 1
        double d2 = 0; // link offset for joint 2
        double d3 = 0; // link offset for joint 3
        double a1 = 0.052; // link length for joint 1 in meters
        double a2 = 0.06606; // link length for joint 2 in m
        double a3 = 0.16871; // link length for joint 3 in m 0.16871
        double alpha1 = M_PI / 2; // link twist for joint 1
        double alpha2 = 0.0; // link twist for joint 2
        double alpha3 = 0.0; // link twist for joint 3

        // those offsets are used for the rest position of the leg
        double theta1_offset = 0.0; // joint angle offset for joint 1
        double theta2_offset = -12.76 * M_PI / 180.0; // joint angle offset for joint 2 (in radians)
        double theta3_offset = -66 * M_PI / 180.0; // joint angle offset for joint 3 (in radians)
      } dh_params_;

    LegTransformMatrix leg_transforms_[6]; // Transform matrices for each leg
    double cycle_duration_ = 0.0; // Duration of one gait cycle in seconds

    const double STRIDE_LENGTH = 0.04; // 4 cm stride
    const double LATERAL_SWAY = 0.04; // 4 cm lateral sway
    const double STEP_HEIGHT = 0.02; // 2 cm step lift
    const double DUTY_CYCLE = 0.5; // stance 50%, swing 50%

    const double COXA_LENGTH = dh_params_.a1;
    const double FEMUR_LENGTH = dh_params_.a2;
    const double TIBIA_LENGTH = dh_params_.a3;
    const double Z_OFFSET = 0.18; // vertical offset from ground to coxa

    /// @brief Converts Denavit-Hartenberg parameters to a transformation matrix
    /// @param theta Joint angle
    /// @param d Link offset
    /// @param a Link length
    /// @param alpha Link twist
    /// @return Denavit-Hartenberg matrix
    Eigen::Matrix4d dhToTransform(double theta, double d, double a, double alpha);

    /// @brief Computes the inverse kinematics for a leg
    /// @param leg_tip_position The position of the leg tip in the base frame
    /// @return The joint angles for the leg
    std::optional<std::array<double, 3>> computeLegIK(Vec3 leg_tip_position);

    double getLegPhaseOffset(int leg_id) const;

    Vec3 computeFootPosition(int leg_id, double time);
};