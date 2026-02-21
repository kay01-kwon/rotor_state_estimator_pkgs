#ifndef RTS_SMOOTHER_HPP
#define RTS_SMOOTHER_HPP

#include "rotor_state_estimator/utils/state_def.hpp"
#include <vector>

/**
 * @brief Rauch-Tung-Striebel (RTS) fixed-lag smoother.
 *
 * Runs a backward pass over a sliding window of forward-filtered CKF results
 * to eliminate phase lag in state estimates. This is the Extended RTS variant
 * that uses the linearized transition matrices from the CKF.
 *
 * Usage:
 *   1. After each CKF predict()+update() cycle, call push_result().
 *   2. Call smooth() to run the backward pass.
 *   3. Retrieve smoothed state/covariance via get_smoothed_state/covariance().
 */
class RtsSmoother
{
public:

    /**
     * @brief Construct a new RTS Smoother.
     *
     * @param window_size Number of forward-pass steps to keep for smoothing.
     * @param motor_params Motor parameters for constraint clipping (optional).
     */
    RtsSmoother(size_t window_size = 20,
                const SecondOrderMotorParams& motor_params = SecondOrderMotorParams());

    ~RtsSmoother();

    /**
     * @brief Push a new CKF forward-pass result into the sliding window.
     *
     * @param result The CKF step result (x_filt, P_filt, x_pred, P_pred, A).
     */
    void push_result(const CkfStepResult& result);

    /**
     * @brief Run the RTS backward pass over the current window.
     *
     * After this call, smoothed states and covariances are available
     * via get_smoothed_state() and get_smoothed_covariance().
     */
    void smooth();

    /**
     * @brief Get the smoothed state at a given lag from the most recent sample.
     *
     * @param lag Number of steps back from the most recent sample (0 = latest).
     *            lag=0 returns the filtered (unsmoothed) latest estimate.
     *            lag>0 returns progressively more smoothed estimates.
     * @return Vector2d Smoothed state [omega, alpha].
     */
    Vector2d get_smoothed_state(size_t lag = 0) const;

    /**
     * @brief Get the smoothed covariance at a given lag from the most recent sample.
     *
     * @param lag Number of steps back from the most recent sample (0 = latest).
     * @return Matrix2x2d Smoothed covariance.
     */
    Matrix2x2d get_smoothed_covariance(size_t lag = 0) const;

    /**
     * @brief Get the number of samples currently in the window.
     */
    size_t size() const;

    /**
     * @brief Check if the window has enough samples for smoothing.
     *
     * @param min_samples Minimum number of samples required (default 2).
     */
    bool ready(size_t min_samples = 2) const;

private:

    size_t window_size_;
    SecondOrderMotorParams motor_params_;

    // Circular buffer of forward-pass results
    std::vector<CkfStepResult> forward_buffer_;
    size_t buffer_head_{0};  // Next write position
    size_t buffer_count_{0}; // Number of valid entries

    // Smoothed results (filled by smooth())
    std::vector<Vector2d> x_smooth_;
    std::vector<Matrix2x2d> P_smooth_;
    bool smoothed_{false};

    /**
     * @brief Get the index into forward_buffer_ for the k-th oldest entry.
     * k=0 is the oldest, k=buffer_count_-1 is the newest.
     */
    size_t buffer_index(size_t k) const;
};

#endif // RTS_SMOOTHER_HPP
