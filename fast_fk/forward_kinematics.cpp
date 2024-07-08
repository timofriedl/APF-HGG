#include <Eigen/Dense>
#include <vector>
#include <cmath>

inline Eigen::Matrix4f jointTransform1(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, 0,
                 std::sin(theta),  std::cos(theta), 0, 0,
                 0, 0, 1, 0.333,
                 0, 0, 0, 1;
    return transform;
}
inline Eigen::Matrix4f jointTransform2(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, 0,
                0, 0, 1, 0,
                -std::sin(theta), -std::cos(theta), 0, 0,
                0, 0, 0, 1;
    return transform;
}
inline Eigen::Matrix4f jointTransform3(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, 0,
                0, 0, -1, -0.316,
                std::sin(theta), std::cos(theta), 0, 0,
                0, 0, 0, 1;
    return transform;
}
inline Eigen::Matrix4f jointTransform4(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, 0.0825,
                0, 0, -1, 0,
                std::sin(theta), std::cos(theta), 0, 0,
                0, 0, 0, 1;
    return transform;
}
inline Eigen::Matrix4f jointTransform5(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, -0.0825,
                0, 0, 1, 0.384,
                -std::sin(theta), -std::cos(theta), 0, 0,
                0, 0, 0, 1;
    return transform;
}
inline Eigen::Matrix4f jointTransform6(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, 0,
                0, 0, -1, 0,
                std::sin(theta), std::cos(theta), 0, 0,
                0, 0, 0, 1;
    return transform;
}
inline Eigen::Matrix4f jointTransform7(float theta) {
    Eigen::Matrix4f transform;
    transform << std::cos(theta), -std::sin(theta), 0, 0.088,
                0, 0, -1, 0,
                std::sin(theta), std::cos(theta), 0, 0,
                0, 0, 0, 1;
    return transform;
}

inline Eigen::Matrix4f flangeTransform() {
    constexpr float SQRT_2_HALF = 0.70710678118;
    Eigen::Matrix4f transform;
    transform << SQRT_2_HALF,  SQRT_2_HALF, 0, 0,
                 -SQRT_2_HALF, SQRT_2_HALF, 0, 0,
                 0,            0,           1, 0.245,
                 0,            0,           0, 1;
    return transform;
}

inline void fk_single(const float* joint_angles, float* result) {
    Eigen::VectorXf q = Eigen::Map<const Eigen::VectorXf>(joint_angles, 7);
    Eigen::Matrix4f fk = Eigen::Matrix4f::Identity();

    fk *= jointTransform1(q[0]);
    fk *= jointTransform2(q[1]);
    fk *= jointTransform3(q[2]);
    fk *= jointTransform4(q[3]);
    fk *= jointTransform5(q[4]);
    fk *= jointTransform6(q[5]);
    fk *= jointTransform7(q[6]);
    fk *= flangeTransform();

    Eigen::Map<Eigen::Matrix4f> result_map(result);
    result_map = fk;
}

extern "C" {
    void forward_kinematics(const float* joint_angles, size_t size, float* result) {
        for (size_t i = 0; i < size; ++i) {
            fk_single(joint_angles + i * 7, result + i * 4 * 4);
        }
    }
}

