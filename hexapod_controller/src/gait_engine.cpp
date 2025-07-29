#include "gait_engine.hpp"
#include <cmath>

GaitEngine::GaitEngine() {
    for (int i = 0; i < 6; ++i) {
        leg_transforms_[i].body_to_coxa = Eigen::Isometry3d::Identity();
        leg_transforms_[i].coxa_to_endpoint = Eigen::Isometry3d::Identity();
    }
    frames_initialized_ = false;
}

void GaitEngine::SetLegFrames(const Eigen::Isometry3d& body_to_coxa, const Eigen::Isometry3d& coxa_to_tibia) {
    // Create coordinate correction transform
    // Standard DH uses Z-axis for joint rotation, but your URDF uses X-axis
    // We need to transform from URDF coordinate system to DH coordinate system
    
    Eigen::Isometry3d coord_correction = Eigen::Isometry3d::Identity();
    
    // Rotate around Y by -90° then around Z by -90° to align X-axis joints with Z-axis DH convention
    // This transforms: X_urdf -> Z_dh, Y_urdf -> X_dh, Z_urdf -> Y_dh
    Eigen::AngleAxisd rot_y(-M_PI/2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z(-M_PI/2, Eigen::Vector3d::UnitZ());
    coord_correction.rotate(rot_y * rot_z);

    // Apply coordinate correction to the incoming tf frames
    leg_transforms_[0].body_to_coxa = coord_correction * body_to_coxa * coord_correction.inverse();
    Eigen::Isometry3d c2t = coord_correction * coxa_to_tibia * coord_correction.inverse();

    // Compute the transform from coxa to endpoint (foot tip)
    // This represents the final link from tibia to the foot endpoint
    Eigen::Isometry3d tibia_to_endpoint = Eigen::Isometry3d::Identity();
    tibia_to_endpoint.translate(Eigen::Vector3d(
        dh_params_.a3 * cos(dh_params_.theta3_offset),
        dh_params_.a3 * sin(dh_params_.theta3_offset),
        0
    ));
    tibia_to_endpoint.rotate(Eigen::AngleAxisd(dh_params_.theta3_offset, Eigen::Vector3d::UnitZ()));
    leg_transforms_[0].coxa_to_endpoint = c2t * tibia_to_endpoint;

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

    // rotation matrix to convert from Z-axis to X-axis rotation
    // model's joint axis is X, so correction is needed
    Eigen::Matrix4d R_correction, R_y, R_z;

    R_y = Eigen::Matrix4d::Identity();
    R_y.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    R_z = Eigen::Matrix4d::Identity();
    R_z.block<3,3>(0,0) = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    R_correction = R_y * R_z;

    // apply the coordinate transformation
    T = R_correction * T_standard * R_correction.transpose();

    return T;
}

double GaitEngine::getLegPhaseOffset(int leg_id) const{
    // tripod gait — 0.0 for first tripod, 0.5 for second
    static const double offsets[6] = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
    return offsets[leg_id];
}

Vec3 GaitEngine::ComputeFootPosition(int leg_id, double time){
    Vec3 pos;
    const double phase_offset = getLegPhaseOffset(leg_id);
    // scaling time into cycles
    double gait_phase = fmod(time / CYCLE_DURATION + phase_offset, 1.0);

    if (gait_phase < DUTY_CYCLE) {
        // stance phase
        double s = gait_phase / DUTY_CYCLE; // parameter normalized [0.0, 1.0]
        pos.x = STRIDE_LENGTH * (0.5 - s); // from +0.04 to -0.04 m
        pos.y = 0.0;
        pos.z = 0.0;
    }
    else {
        // swing phase
        double s = (gait_phase - DUTY_CYCLE) / (1.0 - DUTY_CYCLE); // parameter normalized [0.0, 1.0]
        pos.x = -STRIDE_LENGTH * (0.5 - s); // from -0.04 to +0.04 m
        double y_pos = STRIDE_LENGTH * (s - 0.5);

        pos.x = 0.01 * cos(M_PI * y_pos / STRIDE_LENGTH); // optional lateral sway
        pos.y = y_pos;
        pos.z = STEP_HEIGHT * sin(M_PI * y_pos / STRIDE_LENGTH); // nice smooth arc
    }

    return pos;
}

std::vector<double> GaitEngine::GetLegTrajectoryPoint(int leg_id, double time) {
    auto joint_angles = computeLegIK(ComputeFootPosition(leg_id, time));

    return std::vector<double>(joint_angles.begin(), joint_angles.end());
}

std::vector<double> GaitEngine::GetLegTrajectoryPoint(Vec3 foot_pos) {
    auto joint_angles = computeLegIK(foot_pos);

    return std::vector<double>(joint_angles.begin(), joint_angles.end());
}

Vec3 GaitEngine::ComputeLegFK(const std::array<double, 3>& joint_angles) {
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

std::array<double, 3> GaitEngine::computeLegIK(Vec3 leg_tip_position) {
    double theta1 = atan2(leg_tip_position.y, leg_tip_position.x);
    double theta2 = atan2(abs(leg_tip_position.z), leg_tip_position.x - COXA_LENGTH);

    double l = sqrt(leg_tip_position.z * leg_tip_position.z + (leg_tip_position.x - COXA_LENGTH) * (leg_tip_position.x - COXA_LENGTH));

    theta2 += acos((sqrt(FEMUR_LENGTH * FEMUR_LENGTH - TIBIA_LENGTH * TIBIA_LENGTH + l * l)) / (2 * FEMUR_LENGTH * l));
    
    double theta3 = acos((FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - l * l) / (2 * TIBIA_LENGTH * FEMUR_LENGTH)) - M_PI;

    return {theta1, theta2, theta3};
}

