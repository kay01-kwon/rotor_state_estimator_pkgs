#ifndef CKF_NODE_HPP
#define CKF_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "rotor_state_estimator/ckf/constrained_kalman_filter.hpp"
#include "rotor_state_estimator/utils/CircularBuffer.hpp"

#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>

#include <rotor_state_msgs/msg/rotor_state.hpp>
#include <rotor_state_msgs/msg/rotor_cov.hpp>

using ros2_libcanard_msgs::msg::HexaCmdRaw;
using ros2_libcanard_msgs::msg::HexaActualRpm;

using rotor_state_msgs::msg::RotorState;
using rotor_state_msgs::msg::RotorCov;

class CkfNode : public rclcpp::Node
{
    public:

    CkfNode();

    ~CkfNode();
    
    private:

    /**
     * @brief Load all parameters from the parameter server.
     * 
     */
    void load_parameters();

    /**
     * @brief Print all loaded parameters to the console.
     * 
     * @param motor_params p1, p2, p3, w_min, w_max, alpha_max, jerk_max
     * @param process_noise_cov Process noise covariance matrix in 2x2 double format
     * @param measurement_noise_cov Measurement noise covariance in double format
     * @param initial_cov Initial covariance matrix in 2x2 double format
     * @param estimation_rate Estimation rate in Hz
     */
    void print_parameters(const SecondOrderMotorParams motor_params,
                          const Matrix2x2d process_noise_cov,
                          const double measurement_noise_cov,
                          const Matrix2x2d initial_cov,
                          const double estimation_rate);

    /**
     * @brief Callback function for receiving commanded hexacopter RPM messages.
     * 
     * @param msg Shared pointer to the received HexaCmdRaw message.
     */
    void hexaCmdRawCallback(const HexaCmdRaw::SharedPtr msg);
    
    /**
     * @brief Callback function for receiving actual hexacopter RPM messages.
     * 
     * @param msg Shared pointer to the received HexaActualRpm message.
     */
    void hexaActualRpmCallback(const HexaActualRpm::SharedPtr msg);

    /**
     * @brief Estimate the rotor states using the constrained Kalman filter.
     * 
     */
    void estimate_rotor_states();

    /**
     * @brief Get the command nearest to the given timestamp.
     * 
     * @param timestamp The timestamp to search for.
     * @return Vector6d The command nearest to the given timestamp.
     */
    Vector6d get_cmd_near_timestamp(const double& timestamp);
    
    /**
     * @brief Perform linear interpolation between two RpmData points.
     * 
     * @param before The RpmData point before the target timestamp.
     * @param after The RpmData point after the target timestamp.
     * @param timestamp The target timestamp for interpolation.
     * @return Vector6d The interpolated RPM values at the target timestamp.
     */
    Vector6d rpm_linear_interpolation(const RpmData& before, const RpmData& after, 
        const double& timestamp);

    /**
     * @brief Timer callback function for the rotor state estimation loop.
     * 
     */
    void RotorStateEstimationLoopCallback();



    ConstrainedKf* ckf_[6];
    
    rclcpp::Subscription<HexaCmdRaw>::SharedPtr hexa_cmd_raw_sub_{nullptr};
    rclcpp::Subscription<HexaActualRpm>::SharedPtr hexa_actual_rpm_sub_{nullptr};

    rclcpp::Publisher<RotorState>::SharedPtr rotor_state_pub_{nullptr};
    rclcpp::Publisher<RotorCov>::SharedPtr rotor_cov_pub_{nullptr};

    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    CircularBuffer<RpmData> rpm_buffer_;
    CircularBuffer<RpmData> cmd_rpm_buffer_;

    Vector6d rpm_cmd_;
    Vector6d rpm_meas_;

    double max_rpm_{9800.0};  // Maximum RPM for normalization
    double max_bit_{8191.0}; // Maximum bit value for normalization
    
    Vector12d state_est_;   // 0 ~ 5: rotor speeds, 6 ~ 11: rotor accelerations
    Vector12d state_cov_diag_;  // 0 ~ 5: rotor speed variances, 6 ~ 11: rotor acceleration variances

    RotorState rotor_state_msg_;
    RotorCov rotor_cov_msg_;

    double estimation_rate_{100.0};    // Estimation rate in Hz

};

#endif