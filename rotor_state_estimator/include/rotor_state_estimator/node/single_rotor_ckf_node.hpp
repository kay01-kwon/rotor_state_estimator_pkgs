#ifndef SINGLE_ROTOR_CKF_NODE_HPP
#define SINGLE_ROTOR_CKF_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "rotor_state_estimator/ckf/constrained_kalman_filter.hpp"
#include "rotor_state_estimator/utils/CircularBuffer.hpp"

#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>

#include <rotor_state_msgs/msg/single_rotor_state.hpp>
#include <rotor_state_msgs/msg/single_rotor_cov.hpp>

using ros2_libcanard_msgs::msg::HexaCmdRaw;
using ros2_libcanard_msgs::msg::HexaActualRpm;

using rotor_state_msgs::msg::SingleRotorState;
using rotor_state_msgs::msg::SingleRotorCov;

class SingleRotorCkfNode : public rclcpp::Node
{
    public:

    SingleRotorCkfNode();

    ~SingleRotorCkfNode();

    private:

    /**
     * @brief Load all parameters from the parameter server.
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
     * Extracts only the rotor at rotor_index_.
     *
     * @param msg Shared pointer to the received HexaCmdRaw message.
     */
    void hexaCmdRawCallback(const HexaCmdRaw::SharedPtr msg);

    /**
     * @brief Callback function for receiving actual hexacopter RPM messages.
     * Extracts only the rotor at rotor_index_.
     *
     * @param msg Shared pointer to the received HexaActualRpm message.
     */
    void hexaActualRpmCallback(const HexaActualRpm::SharedPtr msg);

    /**
     * @brief Estimate the single rotor state using the constrained Kalman filter.
     */
    void estimate_rotor_state();

    /**
     * @brief Get the command nearest to the given timestamp.
     *
     * @param timestamp The timestamp to search for.
     * @return double The command RPM nearest to the given timestamp.
     */
    double get_cmd_near_timestamp(const double& timestamp);

    /**
     * @brief Perform linear interpolation between two SingleRpmData points.
     *
     * @param before The SingleRpmData point before the target timestamp.
     * @param after The SingleRpmData point after the target timestamp.
     * @param timestamp The target timestamp for interpolation.
     * @return double The interpolated RPM value at the target timestamp.
     */
    double rpm_linear_interpolation(const SingleRpmData& before, const SingleRpmData& after,
        const double& timestamp);

    /**
     * @brief Timer callback function for the rotor state estimation loop.
     */
    void RotorStateEstimationLoopCallback();

    /**
     * @brief Convert ROS timestamp to double (seconds).
     *
     * @param stamp ROS timestamp.
     * @return double Time in seconds.
     */
    inline double toSeconds(const builtin_interfaces::msg::Time& stamp) const
    {
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    ConstrainedKf* ckf_{nullptr};

    int rotor_index_{0};  // Index of the rotor to estimate (0-5)

    rclcpp::Subscription<HexaCmdRaw>::SharedPtr hexa_cmd_raw_sub_{nullptr};
    rclcpp::Subscription<HexaActualRpm>::SharedPtr hexa_actual_rpm_sub_{nullptr};

    rclcpp::Publisher<SingleRotorState>::SharedPtr rotor_state_pub_{nullptr};
    rclcpp::Publisher<SingleRotorCov>::SharedPtr rotor_cov_pub_{nullptr};

    rclcpp::TimerBase::SharedPtr rotor_state_estimation_timer_{nullptr};

    CircularBuffer<SingleRpmData> rpm_buffer_;
    CircularBuffer<SingleRpmData> cmd_rpm_buffer_;

    double rpm_cmd_{0.0};
    double rpm_meas_{0.0};

    double max_rpm_{9800.0};  // Maximum RPM for normalization
    double max_bit_{8191.0}; // Maximum bit value for normalization

    double idle_cmd_bit_{2000.0}; // Idle command in bit value

    double state_est_speed_{0.0};
    double state_est_accel_{0.0};
    double state_cov_speed_{0.0};
    double state_cov_accel_{0.0};

    SingleRotorState rotor_state_msg_;
    SingleRotorCov rotor_cov_msg_;

    double estimation_rate_{100.0};    // Estimation rate in Hz

};

#endif
