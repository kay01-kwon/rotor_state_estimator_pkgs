#include "rotor_state_estimator/node/ckf_node.hpp"

CkfNode::CkfNode() : Node("ckf_node")
{
    load_parameters();

    rpm_buffer_.reserve(30);
    cmd_rpm_buffer_.reserve(30);


}

CkfNode::~CkfNode()
{
    // Delete CKF instances
    for (int i = 0; i < 6; ++i)
    {
        delete ckf_[i];
    }
}

void CkfNode::load_parameters()
{
    // Implementation for loading parameters from the parameter server

    // 1. Get Motor parameters for second order motor model
    
    // 1.1 Friction
    this->declare_parameter<double>("motor.p1", 25.16687);
    double p1 = this->get_parameter("motor.p1").as_double();

    // 1.2 Drag
    this->declare_parameter<double>("motor.p2", 0.003933);
    double p2 = this->get_parameter("motor.p2").as_double();

    // 1.3 Stiffness
    this->declare_parameter<double>("motor.p3", 0.001070);
    double p3 = this->get_parameter("motor.p3").as_double();

    // 2. Constraints for motor model
    // 2.1 Minimum rotor speed
    this->declare_parameter<double>("motor.w_min", 2000.0);
    double w_min = this->get_parameter("motor.w_min").as_double();

    // 2.2 Maximum rotor speed
    this->declare_parameter<double>("motor.w_max", 7300.0);
    double w_max = this->get_parameter("motor.w_max").as_double();

    // 2.3 Maximum rotor acceleration
    this->declare_parameter<double>("motor.alpha_max", 15000.0);
    double alpha_max = this->get_parameter("motor.alpha_max").as_double();

    // 2.4 Maximum rotor jerk
    this->declare_parameter<double>("motor.jerk_max", 250000.0);
    double jerk_max = this->get_parameter("motor.jerk_max").as_double();

    // Populate motor parameters struct
    SecondOrderMotorParams motor_params;

    // Assign loaded parameters to the struct
    motor_params.p1 = p1;
    motor_params.p2 = p2;
    motor_params.p3 = p3;
    motor_params.w_min = w_min;
    motor_params.w_max = w_max;
    motor_params.alpha_max = alpha_max;
    motor_params.jerk_max = jerk_max;

    // 3. CKF parameters
    
    // 3.1 Process noise covariance
    this->declare_parameter<std::vector<double>>("ckf.process_noise_cov", {1e-1, 10.0});
    std::vector<double> process_noise_cov_vec = this->get_parameter("ckf.process_noise_cov").as_double_array();
    Matrix2x2d process_noise_cov;
    process_noise_cov << process_noise_cov_vec[0], 0.0,
                         0.0, process_noise_cov_vec[1];
    
    // 3.2 Measurement noise covariance
    this->declare_parameter<double>("ckf.measurement_noise_cov", 255.0);
    double measurement_noise_cov = this->get_parameter("ckf.measurement_noise_cov").as_double();

    // 3.3 Initial state covariance
    this->declare_parameter<std::vector<double>>("ckf.initial_state_cov", {100.0, 1000.0});
    std::vector<double> initial_state_cov_vec = this->get_parameter("ckf.initial_state_cov").as_double_array();
    Matrix2x2d initial_cov;
    initial_cov << initial_state_cov_vec[0], 0.0,
                   0.0, initial_state_cov_vec[1];

    // 4. Estimation rate
    this->declare_parameter<double>("ckf.estimation_rate", 100.0);
    estimation_rate_ = this->get_parameter("estimation_rate").as_double();


    for(size_t i = 0; i < 6; ++i)
    {
        ckf_[i] = new ConstrainedKf(motor_params,
                                    process_noise_cov,
                                    measurement_noise_cov,
                                    initial_cov);
    }

    print_parameters(motor_params,
                     process_noise_cov,
                     measurement_noise_cov,
                     initial_cov,
                     estimation_rate_);

}

void CkfNode::print_parameters(const SecondOrderMotorParams motor_params,
                           const Matrix2x2d process_noise_cov,
                           const double measurement_noise_cov,
                           const Matrix2x2d initial_cov,
                           const double estimation_rate)
{
    RCLCPP_INFO(this->get_logger(), "----- CKF Node Parameters -----");
    RCLCPP_INFO(this->get_logger(), "Motor Parameters:");
    RCLCPP_INFO(this->get_logger(), "  p1 (Friction): %.5f", motor_params.p1);
    RCLCPP_INFO(this->get_logger(), "  p2 (Drag): %.5f", motor_params.p2);
    RCLCPP_INFO(this->get_logger(), "  p3 (Stiffness): %.5f", motor_params.p3);
    RCLCPP_INFO(this->get_logger(), "  w_min (Min Rotor Speed): %.2f RPM", motor_params.w_min);
    RCLCPP_INFO(this->get_logger(), "  w_max (Max Rotor Speed): %.2f RPM", motor_params.w_max);
    RCLCPP_INFO(this->get_logger(), "  alpha_max (Max Rotor Acceleration): %.2f RPM/s", motor_params.alpha_max);
    RCLCPP_INFO(this->get_logger(), "  jerk_max (Max Rotor Jerk): %.2f RPM/s^2", motor_params.jerk_max);

    RCLCPP_INFO(this->get_logger(), "CKF Parameters:");
    RCLCPP_INFO(this->get_logger(), "  Process Noise Covariance: [%.5f, %.5f]", process_noise_cov(0,0), process_noise_cov(1,1));
    RCLCPP_INFO(this->get_logger(), "  Measurement Noise Covariance: %.5f", measurement_noise_cov);
    RCLCPP_INFO(this->get_logger(), "  Initial State Covariance: [%.5f, %.5f]", initial_cov(0,0), initial_cov(1,1));
    RCLCPP_INFO(this->get_logger(), "  Estimation Rate: %.2f Hz", estimation_rate);
    RCLCPP_INFO(this->get_logger(), "--------------------------------");
}

void CkfNode::hexaCmdRawCallback(const HexaCmdRaw::SharedPtr msg)
{
    RpmData cmd_data;
    cmd_data.timestamp = toSeconds(msg->header.stamp);

    for (size_t i = 0; i < 6; ++i)
    {
        cmd_data.rpm[i] = (static_cast<double>(msg->cmd_raw[i]) / max_bit_) * max_rpm_;
    }

    if (cmd_rpm_buffer_.is_full())
    {
        cmd_rpm_buffer_.pop();
    }

    cmd_rpm_buffer_.push_back(cmd_data);
}

void CkfNode::hexaActualRpmCallback(const HexaActualRpm::SharedPtr msg)
{
    RpmData rpm_data;
    rpm_data.timestamp = toSeconds(msg->header.stamp);

    for (size_t i = 0; i < 6; ++i)
    {
        rpm_data.rpm[i] = static_cast<double>(msg->rpm[i]);
    }

    if (rpm_buffer_.is_full())
    {
        rpm_buffer_.pop();
    }

    rpm_buffer_.push_back(rpm_data);

    if (rpm_buffer_.size() >= 2 && cmd_rpm_buffer_.size() >= 2)
    {
        estimate_rotor_states();
    }
}

void CkfNode::estimate_rotor_states()
{
    // Implementation for rotor state estimation using CKF

    size_t rpm_idx_recent = rpm_buffer_.size() - 1;
    size_t rpm_idx_before = rpm_buffer_.size() - 2;

    RpmData rpm_recent = rpm_buffer_.at(rpm_idx_recent).value();
    RpmData rpm_before = rpm_buffer_.at(rpm_idx_before).value();

    Vector6d cmd_before = get_cmd_near_timestamp(rpm_before.timestamp);
    Vector6d cmd_recent = get_cmd_near_timestamp(rpm_recent.timestamp);
    Vector6d cmd_med = (cmd_before + cmd_recent) / 2.0;

    double dt = rpm_recent.timestamp - rpm_before.timestamp;
    
    if (dt <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "Non-positive time difference between RPM measurements.");
        return;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        // Prediction step
        ckf_[i]->predict(cmd_med(i), dt);

        // Update step with the most recent RPM measurement
        ckf_[i]->update(rpm_recent.rpm(i));

        // Store estimated states and covariances
        state_est_(i) = ckf_[i]->get_state_estimate()(0);          // Rotor speed
        state_est_(i + 6) = ckf_[i]->get_state_estimate()(1);      // Rotor acceleration

        state_cov_diag_(i) = ckf_[i]->get_covariance_estimate()(0, 0);      // Rotor speed variance
        state_cov_diag_(i + 6) = ckf_[i]->get_covariance_estimate()(1, 1);  // Rotor acceleration variance
    }

}

Vector6d CkfNode::get_cmd_near_timestamp(const double& timestamp)
{
    if (cmd_rpm_buffer_.is_empty())
    {
        Vector6d cmd_near;
        double idle_cmd_rpm = (idle_cmd_bit_ / max_bit_) * max_rpm_;
        cmd_near << idle_cmd_rpm, idle_cmd_rpm, idle_cmd_rpm,
                    idle_cmd_rpm, idle_cmd_rpm, idle_cmd_rpm;
        return cmd_near;
    }

    // Find the two commands that bracket the timestamp
    std::optional<RpmData> before_data;
    std::optional<RpmData> after_data;

    for (size_t i = 0; i < cmd_rpm_buffer_.size(); ++i)
    {
        auto cmd_data = cmd_rpm_buffer_.at(i);
        if (!cmd_data.has_value())
        {
            continue;
        }

        if (cmd_data->timestamp <= timestamp)
        {
            before_data = cmd_data.value();
        }
        else if (cmd_data->timestamp > timestamp && !after_data.has_value())
        {
            after_data = cmd_data.value();
            break;
        }
    }

    // If we have both before and after, interpolate
    if (before_data.has_value() && after_data.has_value())
    {
        return rpm_linear_interpolation(before_data.value(), after_data.value(), timestamp);
    }

    // If we only have before, return it
    if (before_data.has_value())
    {
        return before_data->rpm;
    }

    // If we only have after, return it
    if (after_data.has_value())
    {
        return after_data->rpm;
    }

    // If nothing found, return zero
    Vector6d cmd_near;
    double idle_cmd_rpm = (idle_cmd_bit_ / max_bit_) * max_rpm_;
    cmd_near << idle_cmd_rpm, idle_cmd_rpm, idle_cmd_rpm,
                idle_cmd_rpm, idle_cmd_rpm, idle_cmd_rpm;
    return cmd_near;
}

Vector6d CkfNode::rpm_linear_interpolation(const RpmData& before, const RpmData& after,
    const double& timestamp)
{
    const double time_diff = after.timestamp - before.timestamp;

    if (time_diff <= 0.0)
    {
        return before.rpm;
    }

    const double factor = (timestamp - before.timestamp) / time_diff;

    Vector6d interpolated_rpm;
    for (size_t i = 0; i < 6; ++i)
    {
        interpolated_rpm[i] = before.rpm[i] + factor * (after.rpm[i] - before.rpm[i]);
    }

    return interpolated_rpm;
}

void CkfNode::RotorStateEstimationLoopCallback()
{
    // Just a placeholder for the timer callback function

    // Update message headers with current time
    rotor_state_msg_.header.stamp = this->now();
    rotor_cov_msg_.header.stamp = this->now();

    for(size_t i = 0; i < 6; ++i)
    {
        rotor_state_msg_.rotor_speed[i] = static_cast<int32_t>(state_est_(i));
        rotor_state_msg_.rotor_acceleration[i] = static_cast<int32_t>(state_est_(i + 6));

        rotor_cov_msg_.diag_cov[i] = state_cov_diag_(i);
        rotor_cov_msg_.diag_cov[i + 6] = state_cov_diag_(i + 6);
    }

    rotor_state_pub_->publish(rotor_state_msg_);
    rotor_cov_pub_->publish(rotor_cov_msg_);

}