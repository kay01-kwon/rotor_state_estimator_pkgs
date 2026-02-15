#include "rotor_state_estimator/node/single_rotor_ckf_node.hpp"

SingleRotorCkfNode::SingleRotorCkfNode() : Node("single_rotor_ckf_node")
{
    load_parameters();

    rpm_buffer_.reserve(30);
    cmd_rpm_buffer_.reserve(30);

    std::string cmd_raw_topic = "/uav/cmd_raw";
    std::string actual_rpm_topic = "/uav/actual_rpm";
    std::string rotor_state_topic = "/uav/single_rotor_state";
    std::string rotor_cov_topic = "/uav/single_rotor_state_covariance";

    // Declare and get topic names from parameters
    this->declare_parameter<std::string>("topics.hexa_cmd_raw", cmd_raw_topic);
    this->declare_parameter<std::string>("topics.hexa_actual_rpm", actual_rpm_topic);
    this->declare_parameter<std::string>("topics.single_rotor_state", rotor_state_topic);
    this->declare_parameter<std::string>("topics.single_rotor_cov", rotor_cov_topic);

    // Get topic names from parameters
    cmd_raw_topic = this->get_parameter("topics.hexa_cmd_raw").as_string();
    actual_rpm_topic = this->get_parameter("topics.hexa_actual_rpm").as_string();
    rotor_state_topic = this->get_parameter("topics.single_rotor_state").as_string();
    rotor_cov_topic = this->get_parameter("topics.single_rotor_cov").as_string();

    // Subscribers
    hexa_cmd_raw_sub_ = this->create_subscription<SingleCmdRaw>(
        cmd_raw_topic,
        rclcpp::QoS(5),
        std::bind(&SingleRotorCkfNode::singleCmdRawCallback, this, std::placeholders::_1)
    );

    hexa_actual_rpm_sub_ = this->create_subscription<SingleActualRpm>(
        actual_rpm_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&SingleRotorCkfNode::singleActualRpmCallback, this, std::placeholders::_1)
    );

    // Timer for rotor state estimation loop
    double timer_period = 1.0 / estimation_rate_; // seconds
    rotor_state_estimation_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period),
        std::bind(&SingleRotorCkfNode::RotorStateEstimationLoopCallback, this)
    );

    // Publishers
    rotor_state_pub_ = this->create_publisher<SingleActualRpm>(
        rotor_state_topic,
        rclcpp::SensorDataQoS()
    );

    rotor_cov_pub_ = this->create_publisher<SingleRotorCov>(
        rotor_cov_topic,
        rclcpp::QoS(10)
    );

}

SingleRotorCkfNode::~SingleRotorCkfNode()
{
    delete ckf_;
}

void SingleRotorCkfNode::load_parameters()
{
    // 0. Rotor index parameter
    this->declare_parameter<int>("motor.rotor_index", 0);
    rotor_index_ = this->get_parameter("motor.rotor_index").as_int();

    if (rotor_index_ < 0 || rotor_index_ > 5)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid rotor_index %d. Must be 0-5. Defaulting to 0.", rotor_index_);
        rotor_index_ = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Using rotor index: %d", rotor_index_);

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
    estimation_rate_ = this->get_parameter("ckf.estimation_rate").as_double();

    // Create single CKF instance
    RCLCPP_INFO(this->get_logger(), "Creating CKF instance for rotor %d", rotor_index_);
    ckf_ = new ConstrainedKf(motor_params,
                              process_noise_cov,
                              measurement_noise_cov,
                              initial_cov);

    // Initialize rotor state with idle command RPM
    double w_rotor_init = idle_cmd_bit_ / max_bit_ * max_rpm_;
    RCLCPP_INFO(this->get_logger(), "Initializing rotor %d state with %.2f RPM", rotor_index_, w_rotor_init);

    ckf_->initialize_state(w_rotor_init);

    print_parameters(motor_params,
                     process_noise_cov,
                     measurement_noise_cov,
                     initial_cov,
                     estimation_rate_);

}

void SingleRotorCkfNode::print_parameters(const SecondOrderMotorParams motor_params,
                       const Matrix2x2d process_noise_cov,
                       const double measurement_noise_cov,
                       const Matrix2x2d initial_cov,
                       const double estimation_rate)
{
    RCLCPP_INFO(this->get_logger(), "----- Single Rotor CKF Node Parameters -----");
    RCLCPP_INFO(this->get_logger(), "Rotor Index: %d", rotor_index_);
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
    RCLCPP_INFO(this->get_logger(), "----------------------------------------------");
}

void SingleRotorCkfNode::singleCmdRawCallback(const SingleCmdRaw::SharedPtr msg)
{
    SingleRpmData cmd_data;
    cmd_data.timestamp = toSeconds(msg->header.stamp);

    // Extract only the target rotor's command
    cmd_data.rpm = (static_cast<double>(msg->cmd_raw) / max_bit_) * max_rpm_;

    if (cmd_rpm_buffer_.is_full())
    {
        cmd_rpm_buffer_.pop();
    }

    cmd_rpm_buffer_.push_back(cmd_data);
}

void SingleRotorCkfNode::singleActualRpmCallback(const SingleActualRpm::SharedPtr msg)
{
    SingleRpmData rpm_data;
    rpm_data.timestamp = toSeconds(msg->header.stamp);

    // Extract only the target rotor's RPM
    rpm_data.rpm = static_cast<double>(msg->rpm);

    if (rpm_buffer_.is_full())
    {
        rpm_buffer_.pop();
    }

    rpm_buffer_.push_back(rpm_data);

    if (rpm_buffer_.size() >= 2 && cmd_rpm_buffer_.size() >= 2)
    {
        estimate_rotor_state();
    }
}

void SingleRotorCkfNode::estimate_rotor_state()
{
    size_t rpm_idx_recent = rpm_buffer_.size() - 1;
    size_t rpm_idx_before = rpm_buffer_.size() - 2;

    SingleRpmData rpm_recent = rpm_buffer_.at(rpm_idx_recent).value();
    SingleRpmData rpm_before = rpm_buffer_.at(rpm_idx_before).value();

    double cmd_before = get_cmd_near_timestamp(rpm_before.timestamp);
    double cmd_recent = get_cmd_near_timestamp(rpm_recent.timestamp);
    double cmd_med = (cmd_before + cmd_recent) / 2.0;

    double dt = rpm_recent.timestamp - rpm_before.timestamp;

    if (dt <= 0.0)
    {
        RCLCPP_WARN(this->get_logger(), "Non-positive time difference between RPM measurements.");
        return;
    }

    // Prediction step
    ckf_->predict(cmd_med, dt);

    // Update step with the most recent RPM measurement
    ckf_->update(rpm_recent.rpm);

    // Store estimated states and covariances
    state_est_speed_ = ckf_->get_state_estimate()(0);
    state_est_accel_ = ckf_->get_state_estimate()(1);

    state_cov_speed_ = ckf_->get_covariance_estimate()(0, 0);
    state_cov_accel_ = ckf_->get_covariance_estimate()(1, 1);
}

double SingleRotorCkfNode::get_cmd_near_timestamp(const double& timestamp)
{
    if (cmd_rpm_buffer_.is_empty())
    {
        double idle_cmd_rpm = (idle_cmd_bit_ / max_bit_) * max_rpm_;
        return idle_cmd_rpm;
    }

    // Find the two commands that bracket the timestamp
    std::optional<SingleRpmData> before_data;
    std::optional<SingleRpmData> after_data;

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

    // If nothing found, return idle command RPM
    double idle_cmd_rpm = (idle_cmd_bit_ / max_bit_) * max_rpm_;

    RCLCPP_INFO(this->get_logger(), "No command RPM data found near timestamp %.6f. Returning idle command RPM.", timestamp);
    return idle_cmd_rpm;
}

double SingleRotorCkfNode::rpm_linear_interpolation(const SingleRpmData& before, const SingleRpmData& after,
    const double& timestamp)
{
    const double time_diff = after.timestamp - before.timestamp;

    if (time_diff <= 0.0)
    {
        return before.rpm;
    }

    const double factor = (timestamp - before.timestamp) / time_diff;

    return before.rpm + factor * (after.rpm - before.rpm);
}

void SingleRotorCkfNode::RotorStateEstimationLoopCallback()
{
    // Update message headers with current time
    rotor_state_msg_.header.stamp = this->now();
    rotor_cov_msg_.header.stamp = this->now();

    rotor_state_msg_.rpm = static_cast<int32_t>(state_est_speed_);
    rotor_state_msg_.acceleration = static_cast<int32_t>(state_est_accel_);

    rotor_cov_msg_.diag_cov[0] = state_cov_speed_;
    rotor_cov_msg_.diag_cov[1] = state_cov_accel_;

    rotor_state_pub_->publish(rotor_state_msg_);
    rotor_cov_pub_->publish(rotor_cov_msg_);

}
