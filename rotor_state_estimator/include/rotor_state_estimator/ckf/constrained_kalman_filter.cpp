#include "rotor_state_estimator/ckf/constrained_kalman_filter.hpp"

ConstrainedKf::ConstrainedKf()
{
    // Default constructor

}

ConstrainedKf::ConstrainedKf(const SecondOrderMotorParams& motor_params,
                             const Matrix2x2d& process_noise_cov,
                             const double& measurement_noise_cov,
                             const Matrix2x2d& initial_cov)
{
    load_parameters(motor_params, 
        process_noise_cov, 
        measurement_noise_cov, 
        initial_cov);
}

ConstrainedKf::~ConstrainedKf()
{
    // Destructor
}

bool ConstrainedKf::load_parameters(const SecondOrderMotorParams& motor_params,
                                    const Matrix2x2d& process_noise_cov,
                                    const double& measurement_noise_cov,
                                    const Matrix2x2d& initial_cov)
{

    if(!loaded_params_)
    {
        // Load filter parameters
        motor_params_ = motor_params;
        Q_ = process_noise_cov;
        R_ = measurement_noise_cov;
        P_ = initial_cov;

        // Constraint matrices
        D_ << 1.0, 0.0,     // w <= w_max
              -1.0, 0.0,    // -w <= -w_min
              0.0, 1.0,     // alpha <= alpha_max
              0.0, -1.0;    // -alpha <= alpha_max
        
        // Constraint bounds
        d_ << motor_params_.w_max,
        -motor_params_.w_min,
        motor_params_.alpha_max,
        motor_params_.alpha_max;

        loaded_params_ = true;
        return true;
    }
    else
    {
        loaded_params_ = true;
        std::cerr << "[ConstrainedKf] Parameters already loaded. Ignoring new parameters." << std::endl;
        return false;
    }
}

bool ConstrainedKf::initialize_state(const double& initial_w)
{
    if(!initialized_state_)
    {
        // Initialize state: first 6 elements are angular velocities, 
        // last 6 are angular accelerations set to zeros
        state_estimate_(0) = initial_w;
        state_estimate_(1) = 0.0;

        state_prediction_ = state_estimate_;
        u_cmd_ = initial_w;

        // Initialize state transition and measurement matrices
        A_.setZero();
        H_ << 1.0, 0.0;
        eye_2_.setIdentity();

        initialized_state_ = true;
        return true;
    }
    else
    {
        std::cerr << "[ConstrainedKf] State already initialized. Ignoring new initial state." << std::endl;
        return false;
    }

}

void ConstrainedKf::predict(const double& u_cmd, const double &dt)
{
    // Prediction step implementation goes here
    // Update state estimate and covariance matrix

    // Check if parameters are loaded and state is initialized
    if(!loaded_params_)
    {
        std::cerr << "[ConstrainedKf] Parameters not loaded. Cannot perform prediction." << std::endl;
        return;
    }

    // Check if state is initialized
    if(!initialized_state_)
    {
        std::cerr << "[ConstrainedKf] State not initialized. Cannot perform prediction." << std::endl;
        return;
    }

    double j_eff = 0.0;
    double alpha_eff = 0.0;

    double w_est_prev = state_estimate_(0);
    double alpha_est_prev = state_estimate_(1);

    // Compute physical jerk based on motor model
    j_eff = -(motor_params_.p1 + motor_params_.p2 * w_est_prev) * alpha_est_prev
    - motor_params_.p3 * (w_est_prev - u_cmd);

    // Apply jerk constraints
    j_eff = std::clamp(j_eff, 
        -motor_params_.jerk_max, 
        motor_params_.jerk_max);

    // Check saturation of angular acceleration
    if(alpha_est_prev >= motor_params_.alpha_max && j_eff > 0)
    {
        j_eff = 0.0;
        alpha_eff = motor_params_.alpha_max;
        A_(0,0) = 1.0;
        A_(0,1) = 0.0;
        A_(1,0) = 0.0;
        A_(1,1) = 0.0;
        
    }
    else if(alpha_est_prev <= -motor_params_.alpha_max && j_eff < 0)
    {
        j_eff = 0.0;
        alpha_eff = -motor_params_.alpha_max;
        A_(0,0) = 1.0;
        A_(0,1) = 0.0;
        A_(1,0) = 0.0;
        A_(1,1) = 0.0;
    }
    else
    {
        // No saturation, update angular acceleration
        alpha_eff = alpha_est_prev;

        // Linearized state transition matrix entries
        A_(0,0) = 1.0;
        A_(0,1) = dt;
        A_(1,0) = (-motor_params_.p2*alpha_est_prev - motor_params_.p3) * dt;
        A_(1,1) = 1.0 - (motor_params_.p1 + motor_params_.p2 * w_est_prev) * dt;
    }

    // Update state prediction
    // Update rotor speed
    state_prediction_(0) = w_est_prev 
                        + alpha_eff * dt 
                        + 0.5 * j_eff * dt * dt;
    // Update rotor acceleration
    state_prediction_(1) = alpha_eff + j_eff * dt;
    // Ensure acceleration stays within bounds after prediction
    state_prediction_(1) = std::clamp(state_prediction_(1), 
                                    -motor_params_.alpha_max, 
                                    motor_params_.alpha_max);

    // Update Covariance matrix
    P_ = A_ * P_ * A_.transpose() + Q_;

    // Save predicted covariance for RTS smoother
    P_pred_ = P_;
}

void ConstrainedKf::update(const double& measurement)
{
    // Update step implementation goes here
    // Incorporate measurement into state estimate and covariance matrix

    if(!loaded_params_)
    {
        std::cerr << "[ConstrainedKf] Parameters not loaded. Cannot perform update." << std::endl;
        return;
    }

    if(!initialized_state_)
    {
        std::cerr << "[ConstrainedKf] State not initialized. Cannot perform update." << std::endl;
        return;
    }
    
    // Innovation
    double inov = measurement - H_ * state_prediction_;

    // Innovation covariance
    double S = H_ * P_ * H_.transpose() + R_;

    // Kalman gain
    Eigen::Matrix<double, 2, 1> K = P_ * H_.transpose() / S;

    // Update state estimate
    state_estimate_ = state_prediction_ + K * inov;

    // Update covariance matrix (Joseph form)
    P_ = (eye_2_ - K * H_) * P_ * (eye_2_ - K * H_).transpose() + K * R_ * K.transpose();

    // Check violations of constraints and correct state estimate if necessary
    // (Implementation of constraint handling goes here)

    // Iterate twice to ensure all the constraints are satisfied
    for (int i = 0; i < 2; ++i)
    {
        Vector4d constraint_violations = D_ * state_estimate_ - d_;
        
        // Find the most violated constraint
        Eigen::Index max_index;
        double max_violation = constraint_violations.maxCoeff(&max_index);
        
        if (max_violation <= 0)
        {
            // All constraints are satisfied
            break;
        }

        // Extract the row of D_ corresponding to the most violated constraint
        Matrix1x2d D_i = D_.row(max_index);

        // Maximum A Posteriori (MAP) correction
        double den = D_i * P_ * D_i.transpose();
        double alpha_proj = (D_i * state_estimate_ - d_(max_index)) / den;
        state_estimate_ = state_estimate_ - P_ * D_i.transpose() * alpha_proj;

        if (den > 1e-6)
        {
            // Update covariance matrix
            P_ = P_ - (P_ * D_i.transpose() * D_i * P_) / den;
        }

        P_ = 0.5 * (P_ + P_.transpose()); // Ensure symmetry

        Eigen::EigenSolver<Matrix2x2d> es(P_);
        auto eigenvalues = es.eigenvalues();
        Matrix2x2d D_eigen;
        D_eigen << std::max(eigenvalues(0).real(), 1e-6), 0,
                    0, std::max(eigenvalues(1).real(), 1e-6);
        Matrix2x2d V = es.eigenvectors().real();
        P_ = V * D_eigen * V.transpose();

    }
}

Matrix2x2d ConstrainedKf::get_covariance_estimate() const
{
    return P_;
}

Vector2d ConstrainedKf::get_state_estimate() const
{
    return state_estimate_;
}

Vector2d ConstrainedKf::get_predicted_state() const
{
    return state_prediction_;
}

Matrix2x2d ConstrainedKf::get_predicted_covariance() const
{
    return P_pred_;
}

Matrix2x2d ConstrainedKf::get_transition_matrix() const
{
    return A_;
}