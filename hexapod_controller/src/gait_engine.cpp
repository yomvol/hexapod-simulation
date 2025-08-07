#include "gait_engine.hpp"
#include <cmath>

GaitEngine::GaitEngine(int cycle_duration) {
    for (int i = 0; i < 6; ++i) {
        leg_transforms_[i].body_to_coxa = Eigen::Isometry3d::Identity();
    }
    frames_initialized_ = false;
    cycle_duration_ = cycle_duration / 1000.0;
}

void GaitEngine::setLegFrames(const Eigen::Isometry3d& body_to_coxa, const Eigen::Isometry3d& coxa_to_tibia) {
    leg_transforms_[0].body_to_coxa = body_to_coxa;
    frames_initialized_ = true;
}

Vec3 GaitEngine::transformPoint(const Vec3& point, const Eigen::Isometry3d& transform) const {
    Eigen::Vector3d eigen_point(point.x, point.y, point.z);
    Eigen::Vector3d transformed = transform * eigen_point;
    
    return {transformed.x(), transformed.y(), transformed.z()};
}

Eigen::Matrix4d GaitEngine::dhToTransform(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T;

    // standard DH for Z-axis rotation
    Eigen::Matrix4d T_standard;
    T_standard << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
                  sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                  0,          sin(alpha),             cos(alpha),             d,
                  0,          0,                      0,                      1;

    return T_standard;
}

double GaitEngine::getLegPhaseOffset(int leg_id) const{
    // tripod gait — 0.0 for first tripod, 0.5 for second
    static const double offsets[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
    return offsets[leg_id];
}

Vec3 GaitEngine::computeFootPosition(int leg_id, double time){
    Vec3 relative_stride;
    const double phase_offset = getLegPhaseOffset(leg_id - 1);
    // scaling time into cycles
    double gait_phase = fmod(time / cycle_duration_ + phase_offset, 1.0);

    if (gait_phase < DUTY_CYCLE) {
        // stance phase
        double s = gait_phase / DUTY_CYCLE; // parameter normalized [0.0, 1.0]
        relative_stride.x = STRIDE_LENGTH * (-s); // from 0 to -STRIDE_LENGTH
        relative_stride.z = 0.0;
    }
    else {
        // swing phase
        double s = (gait_phase - DUTY_CYCLE) / (1.0 - DUTY_CYCLE); // parameter normalized [0.0, 1.0]
        relative_stride.x = STRIDE_LENGTH * (s - 1); // from -STRIDE_LENGTH to 0

        //relative_stride.x = 0.01 * cos(M_PI * s); // optional lateral sway (s - 0.5) ???
        relative_stride.z = STEP_HEIGHT * sin(M_PI * s); // nice smooth arc
    }
    relative_stride.y = 0;

    Eigen::Vector3d foot_pos_local_to_coxa = getLegEndpoint(); // relative to coxa frame, Z up (joint axis), x along the leg, y to the side (global coords 0.23, 0.17, -0.18)
    Eigen::Vector3d foot_pos_with_stride = foot_pos_local_to_coxa +
    Eigen::Vector3d(relative_stride.x, relative_stride.y, relative_stride.z); // it's still in leg plane, but robot must move straight

    Eigen::Matrix4d alignment = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    alignment.block<3, 3>(0, 0) = rotation_matrix;
    alignment.block<3, 1>(0, 3) = rotation_matrix * (-foot_pos_local_to_coxa) + foot_pos_local_to_coxa;

    auto foot_pos_aligned = alignment * Eigen::Vector4d(foot_pos_with_stride.x(), foot_pos_with_stride.y(), foot_pos_with_stride.z(), 1.0);
    return {foot_pos_aligned(0), foot_pos_aligned(1), foot_pos_aligned(2)};
}

std::optional<std::vector<double>> GaitEngine::getLegTrajectoryPoint(int leg_id, double time) {
    auto joint_angles = computeLegIK(computeFootPosition(leg_id, time));
    if (!joint_angles) {
        return std::nullopt;
    }

    return std::vector<double>(joint_angles->begin(), joint_angles->end());
}

std::optional<std::vector<double>> GaitEngine::getLegTrajectoryPoint(int leg_id, double time, Vec3& foot_pos_global) {
    auto foot_pos_local_to_coxa = computeFootPosition(leg_id, time);
    
    // rotation matrix to convert from Z-axis to X-axis rotation
    // model's joint axis is X, so correction is needed
    Eigen::Matrix4d R_correction, R_y, R_z, T_local, T_global;

    T_local = Eigen::Isometry3d(Eigen::Translation3d(foot_pos_local_to_coxa.x, foot_pos_local_to_coxa.y, foot_pos_local_to_coxa.z)).matrix();

    R_y = Eigen::Matrix4d::Identity();
    R_y.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    R_z = Eigen::Matrix4d::Identity();
    R_z.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    R_correction = R_y * R_z;

    // apply the coordinate transformation
    T_global = R_correction * T_local * R_correction.inverse();
    auto pos = leg_transforms_[leg_id - 1].body_to_coxa.matrix() * T_global;
    foot_pos_global.x = pos(0, 3);
    foot_pos_global.y = pos(1, 3);
    foot_pos_global.z = pos(2, 3);

    auto joint_angles = computeLegIK(foot_pos_local_to_coxa);
    if (!joint_angles) {
        return std::nullopt;
    }

    return std::vector<double>(joint_angles->begin(), joint_angles->end());
}

Vec3 GaitEngine::computeLegFK(const std::array<double, 3>& joint_angles) {
    Vec3 position;

    Eigen::Matrix4d T0_1, T1_2, T2_3, T0_3;
    T0_1 = dhToTransform(joint_angles[0] + dh_params_.theta1_offset, dh_params_.d1, dh_params_.a1, dh_params_.alpha1);
    T1_2 = dhToTransform(joint_angles[1] + dh_params_.theta2_offset, dh_params_.d2, dh_params_.a2, dh_params_.alpha2);
    T2_3 = dhToTransform(joint_angles[2] + dh_params_.theta3_offset, dh_params_.d3, dh_params_.a3, dh_params_.alpha3);
    T0_3 = T0_1 * T1_2 * T2_3;

    // leg tip position is in the last column (first 3 elements) of the 4x4 matrix
    // however, it's in the leg's local coordinate system
    position.x = T0_3(0, 3);
    position.y = T0_3(1, 3);
    position.z = T0_3(2, 3);

    return position;
}

std::optional<std::array<double, 3>> GaitEngine::computeLegIK(Vec3 leg_tip_position) {
    double theta1 = atan2(leg_tip_position.y, leg_tip_position.x);
    double l = sqrt(Z_OFFSET * Z_OFFSET + (leg_tip_position.x - COXA_LENGTH) * (leg_tip_position.x - COXA_LENGTH));

    // check if target is reachable
    double max_reach = FEMUR_LENGTH + TIBIA_LENGTH;
    if (l > max_reach) return std::nullopt;

    double alpha = atan2(Z_OFFSET, leg_tip_position.x - COXA_LENGTH);

    double cos_beta = (FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH + l * l) / (2 * FEMUR_LENGTH * l);
    cos_beta = std::max(-1.0, std::min(1.0, cos_beta)); // clamp to [-1, 1] to avoid NaN
    double beta = acos(cos_beta);
    double theta2 = alpha + beta - 136.66 * M_PI / 180.0; // don't touch magic numbers lol

    double cos_theta3 = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - l * l) / (2 * TIBIA_LENGTH * FEMUR_LENGTH); // minus pi???
    cos_theta3 = std::max(-1.0, std::min(1.0, cos_theta3)); // clamp to [-1, 1] to avoid NaN
    double theta3 = acos(cos_theta3) - 96.29 * M_PI / 180.0;

    std::array<double, 3> joint_angles = {theta1, theta2, theta3};
    return joint_angles;
}
