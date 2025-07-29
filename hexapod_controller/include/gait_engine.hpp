#pragma once
#include <vector>
#include <array>
#include "Eigen/Dense"
#include "Eigen/Geometry"

struct Vec3 {
  double x, y, z;
};

struct LegTransformMatrix {
    Eigen::Isometry3d body_to_coxa;
    Eigen::Isometry3d coxa_to_endpoint;
};

class GaitEngine {
public:
    GaitEngine();

    /// @brief Computes one point of the given leg's trajectory
    /// @param leg_id The ID of the leg (1-6)
    /// @param time The time at which to compute the IK
    /// @return The joint angles for the leg
    std::vector<double> GetLegTrajectoryPoint(int leg_id, double time);

    /// @brief Computes one point of the given leg's trajectory
    /// @param foot_pos The position of the foot
    /// @return The joint angles for the leg
    std::vector<double> GetLegTrajectoryPoint(Vec3 foot_pos);

    Vec3 ComputeFootPosition(int leg_id, double time);

    /// @brief Computes the forward kinematics for a leg
    /// @param joint_angles The joint angles for the leg
    /// @return The position of the end effector
    Vec3 ComputeLegFK(const std::array<double, 3>& joint_angles);

    /// @brief Initialize with leg frame transforms relative to body
    /// @param body_to_coxa Transform from body frame to coxa frame
    /// @param coxa_to_tibia Transform from coxa frame to tibia frame  
    void SetLegFrames(const Eigen::Isometry3d& body_to_coxa, const Eigen::Isometry3d& coxa_to_tibia);

    /// @brief Get leg endpoint for debug purposes
    /// @param leg_id The ID of the leg (0-5)
    /// @return The position of the leg endpoint in the body frame
    Vec3 GetLegEndpoint(int leg_id) const {
        if (!frames_initialized_) {
            throw std::runtime_error("Leg frames aren't initialized");
        }

        return transformPoint({0, 0, 0}, leg_transforms_[leg_id].body_to_coxa * leg_transforms_[leg_id].coxa_to_endpoint);
    }

  private:
    LegTransformMatrix leg_transforms_[6]; // Transform matrices for each leg
    bool frames_initialized_ = false;
    
    /// @brief Transform a point using Eigen transform
    Vec3 transformPoint(const Vec3& point, const Eigen::Isometry3d& transform) const;

  /// @brief Converts Denavit-Hartenberg parameters to a transformation matrix
  /// @param theta Joint angle
  /// @param d Link offset
  /// @param a Link length
  /// @param alpha Link twist
  /// @return Denavit-Hartenberg matrix
  Eigen::Matrix4d dhToTransform(double theta, double d, double a, double alpha);

  const struct {
    double d1 = 0; // link offset for joint 1
    double d2 = 0; // link offset for joint 2
    double d3 = 0; // link offset for joint 3
    double a1 = 52; // link length for joint 1 (mm)
    double a2 = 66.06; // link length for joint 2 (mm)
    double a3 = 137.71; // link length for joint 3 (mm)
    double alpha1 = M_PI / 2; // link twist for joint 1
    double alpha2 = 0.0; // link twist for joint 2
    double alpha3 = 0.0; // link twist for joint 3

    // those offsets are used for the rest position of the leg
    double theta1_offset = 0.0; // joint angle offset for joint 1
    double theta2_offset = -12.76 * M_PI / 180.0; // joint angle offset for joint 2 (in radians)
    double theta3_offset = -65.35 * M_PI / 180.0; // joint angle offset for joint 3 (in radians)
  } dh_params_;

  const double STRIDE_LENGTH = 0.08; // 8 cm stride
  const double STEP_HEIGHT = 0.02; // 2 cm step lift
  const double CYCLE_DURATION = 1.0; // 1 sec per cycle
  const double DUTY_CYCLE = 0.5; // stance 50%, swing 50%

  const double COXA_LENGTH = 0.052; // in meters
  const double FEMUR_LENGTH = 0.06606;
  const double TIBIA_LENGTH = 0.13771;

  /// @brief Computes the inverse kinematics for a leg
  /// @param leg_tip_position The position of the leg tip in the base frame
  /// @return The joint angles for the leg
  std::array<double, 3> computeLegIK(Vec3 leg_tip_position);

  double getLegPhaseOffset(int leg_id) const;

};