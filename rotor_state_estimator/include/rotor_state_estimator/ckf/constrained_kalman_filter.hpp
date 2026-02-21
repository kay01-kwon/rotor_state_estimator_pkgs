#ifndef CONSTRAINED_KALMAN_FILTER_HPP
#define CONSTRAINED_KALMAN_FILTER_HPP

#include "rotor_state_estimator/utils/state_def.hpp"
#include <algorithm>

class ConstrainedKf
{
    public:

    /**
     * @brief Default constructor.
     * 
     */
    ConstrainedKf();

    /**
     * @brief Parameterized constructor.
     * 
     * @param motor_params Friction, drag, stiffness coefficients and constraints.
     * @param process_noise_cov Process noise covariance matrix.
     * @param measurement_noise_cov Measurement noise covariance matrix.
     * @param initial_cov Initial covariance matrix.
     */
    ConstrainedKf(const SecondOrderMotorParams& motor_params,
                  const Matrix2x2d& process_noise_cov,
                  const double& measurement_noise_cov,
                  const Matrix2x2d& initial_cov);

    /**
     * @brief Destroy the Constrained Kf object
     * 
     */
    ~ConstrainedKf();

    /**
     * @brief Load all parameters. If not loaded, the filter will not operate.
     * If parameters are already loaded, they will be ignored.
     * Only the first load will be considered.
     * 
     * @param motor_params Friction, drag, stiffness coefficients and constraints.
     * @param process_noise_cov Process noise covariance matrix.
     * @param measurement_noise_cov Measurement noise covariance matrix.
     * @param initial_cov Initial covariance matrix.
     * @return true if parameters are loaded successfully, false otherwise.
     */
    bool load_parameters(const SecondOrderMotorParams& motor_params,
                         const Matrix2x2d& process_noise_cov,
                         const double& measurement_noise_cov,
                         const Matrix2x2d& initial_cov);
    /**
     * @brief Initialize the state vector with initial angular velocities.
     * The angular accelerations are initialized to zero.
     * 
     * @param initial_w Initial angular velocities.
     * @return true if the state is initialized successfully, false if already initialized.
     */
    bool initialize_state(const double& initial_w);

    /**
     * @brief Perform a prediction step of the constrained Kalman filter.
     * 
     */
    void predict(const double& u_cmd, const double &dt);
    
    /**
     * @brief Perform an update step of the constrained Kalman filter.
     * 
     * @param measurement Measurement vector.
     */
    void update(const double& measurement);

    /**
     * @brief Get the current state estimate.
     *
     * @return StateVector Current state estimate vector.
     */
    Vector2d get_state_estimate() const;

    Matrix2x2d get_covariance_estimate() const;

    /**
     * @brief Get the predicted state (before measurement update).
     * Valid after calling predict().
     */
    Vector2d get_predicted_state() const;

    /**
     * @brief Get the predicted covariance (before measurement update).
     * Valid after calling predict().
     */
    Matrix2x2d get_predicted_covariance() const;

    /**
     * @brief Get the linearized state transition matrix.
     * Valid after calling predict().
     */
    Matrix2x2d get_transition_matrix() const;

    private:


    // Filter parameters
    SecondOrderMotorParams motor_params_;   // Motor parameters

    Matrix2x2d Q_;  // Process noise covariance matrix
    double R_;  // Measurement noise covariance matrix

    Vector2d state_prediction_; // Predicted state vector
    Vector2d state_estimate_;    // State estimate vector
    double u_cmd_;    // Control input vector (commanded angular velocities)
    
    Matrix2x2d P_;  // Estimate covariance matrix
    Matrix2x2d P_pred_;  // Predicted covariance (before measurement update)
    Matrix2x2d A_;  // State transition matrix

    Matrix1x2d H_;  // Measurement matrix
    Matrix2x2d eye_2_; // 2x2 Identity matrix

    Matrix4x2d D_;  // Constraint matrix
    Vector4d d_; // constraint vector

    bool loaded_params_{false};
    bool initialized_state_{false};
    bool initialized_covariance_{false};

};


#endif // CONSTRAINED_KALMAN_FILTER_HPP