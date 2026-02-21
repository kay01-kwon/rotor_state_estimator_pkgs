#include "rotor_state_estimator/ckf/rts_smoother.hpp"
#include <algorithm>

RtsSmoother::RtsSmoother(size_t window_size,
                         const SecondOrderMotorParams& motor_params)
    : window_size_(window_size),
      motor_params_(motor_params)
{
    forward_buffer_.resize(window_size_);
    x_smooth_.resize(window_size_);
    P_smooth_.resize(window_size_);
}

RtsSmoother::~RtsSmoother()
{
}

void RtsSmoother::push_result(const CkfStepResult& result)
{
    forward_buffer_[buffer_head_] = result;
    buffer_head_ = (buffer_head_ + 1) % window_size_;

    if (buffer_count_ < window_size_)
    {
        buffer_count_++;
    }

    smoothed_ = false;
}

size_t RtsSmoother::buffer_index(size_t k) const
{
    // k=0 is oldest, k=buffer_count_-1 is newest
    // buffer_head_ points to the next write position (one past newest)
    // newest is at (buffer_head_ - 1 + window_size_) % window_size_
    // oldest is at (buffer_head_ - buffer_count_ + window_size_) % window_size_
    size_t oldest = (buffer_head_ - buffer_count_ + window_size_) % window_size_;
    return (oldest + k) % window_size_;
}

void RtsSmoother::smooth()
{
    if (buffer_count_ < 2)
    {
        // Not enough data to smooth; just copy filtered
        if (buffer_count_ == 1)
        {
            size_t idx = buffer_index(0);
            x_smooth_[0] = forward_buffer_[idx].x_filt;
            P_smooth_[0] = forward_buffer_[idx].P_filt;
        }
        smoothed_ = true;
        return;
    }

    const size_t N = buffer_count_;

    // Initialize with the last (newest) filtered estimate
    size_t last_buf_idx = buffer_index(N - 1);
    x_smooth_[N - 1] = forward_buffer_[last_buf_idx].x_filt;
    P_smooth_[N - 1] = forward_buffer_[last_buf_idx].P_filt;

    // Backward pass: k = N-2 down to 0
    for (int k = static_cast<int>(N) - 2; k >= 0; --k)
    {
        size_t idx_k = buffer_index(static_cast<size_t>(k));
        size_t idx_kp1 = buffer_index(static_cast<size_t>(k + 1));

        const CkfStepResult& fwd_k = forward_buffer_[idx_k];
        const CkfStepResult& fwd_kp1 = forward_buffer_[idx_kp1];

        // Smoother gain: G_k = P_filt[k] * A_{k+1}^T * inv(P_pred[k+1])
        Matrix2x2d P_pred_kp1 = fwd_kp1.P_pred;

        // Compute G_k = P_filt[k] * A_{k+1}^T * P_pred[k+1]^{-1}
        // Use direct inverse; fall back to pseudo-inverse if singular
        double det = P_pred_kp1(0,0) * P_pred_kp1(1,1) - P_pred_kp1(0,1) * P_pred_kp1(1,0);
        Matrix2x2d P_pred_inv;

        if (std::abs(det) > 1e-10)
        {
            // Direct 2x2 inverse
            P_pred_inv(0,0) =  P_pred_kp1(1,1) / det;
            P_pred_inv(0,1) = -P_pred_kp1(0,1) / det;
            P_pred_inv(1,0) = -P_pred_kp1(1,0) / det;
            P_pred_inv(1,1) =  P_pred_kp1(0,0) / det;
        }
        else
        {
            // Pseudo-inverse via Eigen for degenerate case
            Eigen::JacobiSVD<Matrix2x2d> svd(P_pred_kp1,
                Eigen::ComputeFullU | Eigen::ComputeFullV);
            auto S = svd.singularValues();
            Matrix2x2d S_inv = Matrix2x2d::Zero();
            for (int i = 0; i < 2; ++i)
            {
                if (S(i) > 1e-10)
                    S_inv(i, i) = 1.0 / S(i);
            }
            P_pred_inv = svd.matrixV() * S_inv * svd.matrixU().transpose();
        }

        Matrix2x2d G = fwd_k.P_filt * fwd_kp1.A.transpose() * P_pred_inv;

        // Smoothed state
        x_smooth_[k] = fwd_k.x_filt
                        + G * (x_smooth_[k + 1] - fwd_kp1.x_pred);

        // Smoothed covariance
        P_smooth_[k] = fwd_k.P_filt
                        + G * (P_smooth_[k + 1] - P_pred_kp1) * G.transpose();

        // Ensure symmetry and positive-definiteness
        P_smooth_[k] = 0.5 * (P_smooth_[k] + P_smooth_[k].transpose());

        Eigen::EigenSolver<Matrix2x2d> es(P_smooth_[k]);
        auto eigenvalues = es.eigenvalues();
        Matrix2x2d D_eigen;
        D_eigen << std::max(eigenvalues(0).real(), 1e-10), 0,
                   0, std::max(eigenvalues(1).real(), 1e-10);
        Matrix2x2d V = es.eigenvectors().real();
        P_smooth_[k] = V * D_eigen * V.transpose();

        // Clip smoothed state to constraints
        x_smooth_[k](0) = std::clamp(x_smooth_[k](0),
                                       motor_params_.w_min,
                                       motor_params_.w_max);
        x_smooth_[k](1) = std::clamp(x_smooth_[k](1),
                                       -motor_params_.alpha_max,
                                       motor_params_.alpha_max);
    }

    smoothed_ = true;
}

Vector2d RtsSmoother::get_smoothed_state(size_t lag) const
{
    if (!smoothed_ || buffer_count_ == 0)
    {
        return Vector2d::Zero();
    }

    // lag=0 => newest (index N-1), lag=1 => second newest, etc.
    size_t N = buffer_count_;
    if (lag >= N)
    {
        lag = N - 1;
    }

    return x_smooth_[N - 1 - lag];
}

Matrix2x2d RtsSmoother::get_smoothed_covariance(size_t lag) const
{
    if (!smoothed_ || buffer_count_ == 0)
    {
        return Matrix2x2d::Zero();
    }

    size_t N = buffer_count_;
    if (lag >= N)
    {
        lag = N - 1;
    }

    return P_smooth_[N - 1 - lag];
}

size_t RtsSmoother::size() const
{
    return buffer_count_;
}

bool RtsSmoother::ready(size_t min_samples) const
{
    return buffer_count_ >= min_samples;
}
