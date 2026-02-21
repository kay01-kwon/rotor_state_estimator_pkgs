#ifndef STATE_DEF_HPP
#define STATE_DEF_HPP

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Vector and Matrix type definitions

typedef Eigen::Matrix<double, 1, 2> Matrix1x2d;
typedef Eigen::Matrix<double, 1, 4> Matrix1x4d;

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2d;

typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 2> Matrix4x2d;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<int16_t, 6, 1> Vector6int16;

typedef Eigen::Matrix<double, 12, 1> Vector12d;


// Second order motor parameters
struct SecondOrderMotorParams
{
    double p1{25.16687};    // Friction coefficient
    double p2{0.003933};     // Drag coefficient
    double p3{515.650};   // Motor Stiffness

    double w_min{2000.0};        // Minimum angular velocity [RPM]
    double w_max{7300.0};        // Maximum angular velocity [RPM]
    double alpha_max{15000.0};      // Maximum angular acceleration [RPM/s]
    double jerk_max{250000.0};      // Maximum angular jerk [RPM/s^2]
};

struct RpmData
{
    double timestamp{0.0};
    Vector6d rpm{Vector6d::Zero()};
};

struct SingleRpmData
{
    double timestamp{0.0};
    double rpm{0.0};
};

// Forward-pass result stored for RTS smoother backward pass
struct CkfStepResult
{
    Vector2d x_filt{Vector2d::Zero()};      // Filtered state (after update)
    Matrix2x2d P_filt{Matrix2x2d::Zero()};  // Filtered covariance (after update)
    Vector2d x_pred{Vector2d::Zero()};      // Predicted state (before update)
    Matrix2x2d P_pred{Matrix2x2d::Zero()};  // Predicted covariance (before update)
    Matrix2x2d A{Matrix2x2d::Zero()};       // Linearized state transition matrix
};

#endif // STATE_DEF_HPP