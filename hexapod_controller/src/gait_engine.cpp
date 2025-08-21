#include "gait_engine.hpp"
#include <cmath>

GaitEngine::GaitEngine(int cycle_duration) {
    for (int i = 0; i < 6; ++i) {
        leg_transforms_[i].body_to_coxa = Eigen::Isometry3d::Identity();
    }
    cycle_duration_ = cycle_duration;
}

void GaitEngine::setLegFrames(int leg_id, const Eigen::Isometry3d& body_to_coxa) {
    leg_transforms_[leg_id].body_to_coxa = body_to_coxa;
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
    relative_stride.y = 0.0;
    const double phase_offset = getLegPhaseOffset(leg_id);
    double gait_phase = fmod(time / cycle_duration_ + phase_offset, 1.0);
    int sign = (leg_id < 3) ? 1 : -1;

    if (gait_phase < DUTY_CYCLE) {
        // stance phase
        double s = gait_phase / DUTY_CYCLE; // parameter normalized [0.0, 1.0]
        relative_stride.x = STRIDE_LENGTH * (-s); // from 0 to -STRIDE_LENGTH
    }
    else {
        // swing phase
        double s = (gait_phase - DUTY_CYCLE) / (1.0 - DUTY_CYCLE); // parameter normalized [0.0, 1.0]
        relative_stride.x = STRIDE_LENGTH * (s - 1); // from -STRIDE_LENGTH to 0
        double lateral_sway = leg_id == 1 || leg_id == 4 ? 0.02 : 0.04;
        relative_stride.y = sign * lateral_sway * sin(M_PI * s); // lateral sway is needed, otherwise tibia doesn't have enough space
        relative_stride.z = STEP_HEIGHT * sin(M_PI * s); // nice smooth arc
    }
    if (leg_id == 0 || leg_id == 5) relative_stride.y += sign * 0.02;
    if (leg_id == 2 || leg_id == 3) relative_stride.x += 0.02;

    Eigen::Vector3d foot_pos_local_to_coxa = getLegEndpoint(); // relative to coxa frame, Z up (joint axis), x along the leg, y to the side (global coords 0.23, 0.17, -0.18)
    Eigen::Vector3d offset = Eigen::Vector3d(relative_stride.x, relative_stride.y, relative_stride.z);
    double angle = 0.0;
    switch (leg_id) {
        case 0:
            angle = -M_PI / 4;
            break;
        case 1:
            angle = -M_PI / 2;
            break;
        case 2:
            angle = -3 * M_PI / 4;
            break;
        case 3:
            angle = 3 * M_PI / 4;
            break;
        case 4:
            angle = M_PI / 2;
            break;
        case 5:
            angle = M_PI / 4;
            break;
        default:
            throw std::invalid_argument("Invalid leg ID: " + std::to_string(leg_id));
    }
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto foot_pos_aligned = foot_pos_local_to_coxa + rotation_matrix * offset;
    return Vec3(foot_pos_aligned(0), foot_pos_aligned(1), foot_pos_aligned(2));
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
    auto pos = leg_transforms_[leg_id].body_to_coxa.matrix() * T_global;
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
    leg_tip_position.z += Z_OFFSET; // add offset to the Z coordinate, because IK origin is on the ground

    double theta1 = atan2(leg_tip_position.y, leg_tip_position.x);
    double l = sqrt((Z_OFFSET - leg_tip_position.z) * (Z_OFFSET - leg_tip_position.z)
    + (leg_tip_position.x - COXA_LENGTH) * (leg_tip_position.x - COXA_LENGTH));

    // angles theta2 and theta3 are computed in the plane of the leg (coxa to endpoint)
    // r is the distance from the coxa to the endpoint in the XY plane
    double r = sqrt(leg_tip_position.x * leg_tip_position.x + leg_tip_position.y * leg_tip_position.y);

    // check if target is reachable
    double max_reach = FEMUR_LENGTH + TIBIA_LENGTH;
    if (l > max_reach) return std::nullopt;

    double alpha = atan2(Z_OFFSET - leg_tip_position.z, r - COXA_LENGTH);

    double cos_beta = (FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH + l * l) / (2 * FEMUR_LENGTH * l);
    //cos_beta = std::max(-1.0, std::min(1.0, cos_beta)); // clamp to [-1, 1] to avoid NaN
    double beta = acos(cos_beta);
    //double theta2 = M_PI - (alpha + beta); // don't touch magic numbers lol -110.4948
    double theta2 = alpha + beta -110.4948 * M_PI / 180.0;

    double cos_theta3 = (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - l * l) / (2 * TIBIA_LENGTH * FEMUR_LENGTH);
    //cos_theta3 = std::max(-1.0, std::min(1.0, cos_theta3)); // clamp to [-1, 1] to avoid NaN
    //double theta3 = M_PI/2 - acos(cos_theta3); // -114.0014
    double theta3 = acos(cos_theta3) - 114.0014 * M_PI / 180.0;

    std::array<double, 3> joint_angles = {theta1, theta2, theta3};
    return joint_angles;
}
