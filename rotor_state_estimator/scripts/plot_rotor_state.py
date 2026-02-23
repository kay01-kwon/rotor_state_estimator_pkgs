#!/usr/bin/env python3
"""
Real-time plotter for rotor state estimation with 3-sigma bounds.

Subscribes to:
  - /uav/actual_rpm          (SingleActualRpm)  – raw RPM measurement
  - /uav/single_rotor_state  (SingleActualRpm)  – estimated state [rpm, accel]
  - /uav/single_rotor_state_covariance (SingleRotorCov) – diagonal covariance

Plots:
  Top:    RPM measurement vs. RTS-smoothed estimate with ±3σ band
  Bottom: Acceleration estimate with ±3σ band
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ros2_libcanard_msgs.msg import SingleActualRpm
from rotor_state_msgs.msg import SingleRotorCov

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class RotorStatePlotter(Node):

    def __init__(self, max_points=500):
        super().__init__("rotor_state_plotter")

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter("topics.actual_rpm", "/uav/actual_rpm")
        self.declare_parameter("topics.estimated_state", "/uav/single_rotor_state")
        self.declare_parameter("topics.estimated_cov", "/uav/single_rotor_state_covariance")
        self.declare_parameter("max_points", max_points)

        actual_topic = self.get_parameter("topics.actual_rpm").as_string()
        est_topic = self.get_parameter("topics.estimated_state").as_string()
        cov_topic = self.get_parameter("topics.estimated_cov").as_string()
        self.max_pts = self.get_parameter("max_points").as_int()

        # ── Data buffers (deque for O(1) append/pop) ────────────
        self.t_meas = deque(maxlen=self.max_pts)
        self.rpm_meas = deque(maxlen=self.max_pts)

        self.t_est = deque(maxlen=self.max_pts)
        self.rpm_est = deque(maxlen=self.max_pts)
        self.accel_est = deque(maxlen=self.max_pts)

        self.t_cov = deque(maxlen=self.max_pts)
        self.sigma_rpm = deque(maxlen=self.max_pts)
        self.sigma_accel = deque(maxlen=self.max_pts)

        self.t0 = None  # reference time

        # ── QoS profiles ────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ─────────────────────────────────────────
        self.create_subscription(
            SingleActualRpm, actual_topic,
            self._cb_actual, sensor_qos)
        self.create_subscription(
            SingleActualRpm, est_topic,
            self._cb_estimate, sensor_qos)
        self.create_subscription(
            SingleRotorCov, cov_topic,
            self._cb_covariance, reliable_qos)

        self.get_logger().info(
            f"Plotter started: meas={actual_topic}, "
            f"est={est_topic}, cov={cov_topic}")

    # ── Helpers ─────────────────────────────────────────────────
    def _stamp_to_sec(self, stamp):
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    # ── Callbacks ───────────────────────────────────────────────
    def _cb_actual(self, msg):
        t = self._stamp_to_sec(msg.header.stamp)
        self.t_meas.append(t)
        self.rpm_meas.append(float(msg.rpm))

    def _cb_estimate(self, msg):
        t = self._stamp_to_sec(msg.header.stamp)
        self.t_est.append(t)
        self.rpm_est.append(float(msg.rpm))
        self.accel_est.append(float(msg.acceleration))

    def _cb_covariance(self, msg):
        t = self._stamp_to_sec(msg.header.stamp)
        self.t_cov.append(t)
        self.sigma_rpm.append(np.sqrt(max(msg.diag_cov[0], 0.0)))
        self.sigma_accel.append(np.sqrt(max(msg.diag_cov[1], 0.0)))


def main():
    rclpy.init()
    node = RotorStatePlotter()

    # ── Matplotlib setup ────────────────────────────────────────
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, (ax_rpm, ax_acc) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("Rotor State Estimation (RTS Smoother)", fontsize=14)

    # RPM subplot
    (ln_meas,) = ax_rpm.plot([], [], "o", color="gray", markersize=2,
                              alpha=0.5, label="Measured RPM")
    (ln_est,)  = ax_rpm.plot([], [], "-", color="tab:blue", linewidth=1.5,
                              label="Estimated RPM")
    fill_rpm = None

    ax_rpm.set_ylabel("RPM")
    ax_rpm.legend(loc="upper left", fontsize=9)

    # Acceleration subplot
    (ln_acc,) = ax_acc.plot([], [], "-", color="tab:orange", linewidth=1.5,
                             label="Estimated Accel")
    fill_acc = None

    ax_acc.set_ylabel("Acceleration [RPM/s]")
    ax_acc.set_xlabel("Time [s]")
    ax_acc.legend(loc="upper left", fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    # ── Animation update ────────────────────────────────────────
    def update(_frame):
        nonlocal fill_rpm, fill_acc

        # Spin ROS callbacks
        rclpy.spin_once(node, timeout_sec=0.0)

        # --- RPM ---
        if len(node.t_meas) > 0:
            ln_meas.set_data(list(node.t_meas), list(node.rpm_meas))

        t_e = list(node.t_est)
        rpm_e = np.array(node.rpm_est)

        if len(t_e) > 0:
            ln_est.set_data(t_e, rpm_e)

        # 3-sigma fill for RPM
        if fill_rpm is not None:
            fill_rpm.remove()
            fill_rpm = None

        t_c = list(node.t_cov)
        sig_r = np.array(node.sigma_rpm)

        if len(t_c) > 1 and len(t_e) > 1:
            # Align covariance to estimate times via nearest-neighbor
            n = min(len(t_e), len(t_c))
            t_plot = t_e[-n:]
            rpm_plot = rpm_e[-n:]
            sig_plot = sig_r[-n:]
            upper = rpm_plot + 3.0 * sig_plot
            lower = rpm_plot - 3.0 * sig_plot
            fill_rpm = ax_rpm.fill_between(
                t_plot, lower, upper,
                color="tab:blue", alpha=0.15, label="±3σ")

        # --- Acceleration ---
        accel_e = np.array(node.accel_est)

        if len(t_e) > 0:
            ln_acc.set_data(t_e, accel_e)

        if fill_acc is not None:
            fill_acc.remove()
            fill_acc = None

        sig_a = np.array(node.sigma_accel)

        if len(t_c) > 1 and len(t_e) > 1:
            n = min(len(t_e), len(t_c))
            t_plot = t_e[-n:]
            acc_plot = accel_e[-n:]
            sig_plot = sig_a[-n:]
            upper = acc_plot + 3.0 * sig_plot
            lower = acc_plot - 3.0 * sig_plot
            fill_acc = ax_acc.fill_between(
                t_plot, lower, upper,
                color="tab:orange", alpha=0.15, label="±3σ")

        # Auto-scale axes
        for ax in (ax_rpm, ax_acc):
            ax.relim()
            ax.autoscale_view()

        return ln_meas, ln_est, ln_acc

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
