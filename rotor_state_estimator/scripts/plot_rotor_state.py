#!/usr/bin/env python3
"""
Real-time plotter for rotor state estimation with 3-sigma bounds.

Supports two modes via the 'mode' parameter:
  - "single" : one rotor  (SingleActualRpm + SingleRotorCov)
  - "hexa"   : six rotors (HexaActualRpm + RotorCov)  — 3x2 grid
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ── Message imports ─────────────────────────────────────────────
from ros2_libcanard_msgs.msg import SingleActualRpm, HexaActualRpm
from rotor_state_msgs.msg import SingleRotorCov, RotorCov

NUM_ROTORS = 6

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

COLORS = ["tab:blue", "tab:orange", "tab:green",
          "tab:red", "tab:purple", "tab:brown"]


# ====================================================================
#  Single-rotor plotter node
# ====================================================================
class SingleRotorPlotterNode(Node):

    def __init__(self):
        super().__init__("single_rotor_plotter")

        self.declare_parameter("topics.actual_rpm", "/uav/actual_rpm")
        self.declare_parameter("topics.estimated_state",
                               "/uav/single_rotor_state")
        self.declare_parameter("topics.estimated_cov",
                               "/uav/single_rotor_state_covariance")
        self.declare_parameter("max_points", 500)

        actual_topic = self.get_parameter("topics.actual_rpm").value
        est_topic    = self.get_parameter("topics.estimated_state").value
        cov_topic    = self.get_parameter("topics.estimated_cov").value
        self.max_pts = int(self.get_parameter("max_points").value)

        # Buffers
        self.t_meas    = deque(maxlen=self.max_pts)
        self.rpm_meas  = deque(maxlen=self.max_pts)
        self.t_est     = deque(maxlen=self.max_pts)
        self.rpm_est   = deque(maxlen=self.max_pts)
        self.accel_est = deque(maxlen=self.max_pts)
        self.t_cov     = deque(maxlen=self.max_pts)
        self.sig_rpm   = deque(maxlen=self.max_pts)
        self.sig_acc   = deque(maxlen=self.max_pts)
        self.t0 = None

        # Subscribers
        self.create_subscription(SingleActualRpm, actual_topic,
                                 self._cb_meas, SENSOR_QOS)
        self.create_subscription(SingleActualRpm, est_topic,
                                 self._cb_est, SENSOR_QOS)
        self.create_subscription(SingleRotorCov, cov_topic,
                                 self._cb_cov, RELIABLE_QOS)

        self.get_logger().info(
            f"[single] meas={actual_topic}  est={est_topic}  cov={cov_topic}")

    # helpers
    def _sec(self, stamp):
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _cb_meas(self, msg):
        self.t_meas.append(self._sec(msg.header.stamp))
        self.rpm_meas.append(float(msg.rpm))

    def _cb_est(self, msg):
        self.t_est.append(self._sec(msg.header.stamp))
        self.rpm_est.append(float(msg.rpm))
        self.accel_est.append(float(msg.acceleration))

    def _cb_cov(self, msg):
        self.t_cov.append(self._sec(msg.header.stamp))
        self.sig_rpm.append(np.sqrt(max(msg.diag_cov[0], 0.0)))
        self.sig_acc.append(np.sqrt(max(msg.diag_cov[1], 0.0)))


# ====================================================================
#  Hexa-rotor plotter node
# ====================================================================
class HexaRotorPlotterNode(Node):

    def __init__(self):
        super().__init__("hexa_rotor_plotter")

        self.declare_parameter("topics.actual_rpm", "/uav/actual_rpm")
        self.declare_parameter("topics.estimated_state", "/uav/rotor_state")
        self.declare_parameter("topics.estimated_cov",
                               "/uav/rotor_state_covariance")
        self.declare_parameter("max_points", 500)

        actual_topic = self.get_parameter("topics.actual_rpm").value
        est_topic    = self.get_parameter("topics.estimated_state").value
        cov_topic    = self.get_parameter("topics.estimated_cov").value
        self.max_pts = int(self.get_parameter("max_points").value)

        # Per-rotor buffers  (index 0..5)
        self.t_meas   = deque(maxlen=self.max_pts)
        self.rpm_meas = [deque(maxlen=self.max_pts) for _ in range(NUM_ROTORS)]

        self.t_est     = deque(maxlen=self.max_pts)
        self.rpm_est   = [deque(maxlen=self.max_pts) for _ in range(NUM_ROTORS)]
        self.accel_est = [deque(maxlen=self.max_pts) for _ in range(NUM_ROTORS)]

        self.t_cov   = deque(maxlen=self.max_pts)
        self.sig_rpm = [deque(maxlen=self.max_pts) for _ in range(NUM_ROTORS)]
        self.sig_acc = [deque(maxlen=self.max_pts) for _ in range(NUM_ROTORS)]

        self.t0 = None

        # Subscribers
        self.create_subscription(HexaActualRpm, actual_topic,
                                 self._cb_meas, SENSOR_QOS)
        self.create_subscription(HexaActualRpm, est_topic,
                                 self._cb_est, SENSOR_QOS)
        self.create_subscription(RotorCov, cov_topic,
                                 self._cb_cov, RELIABLE_QOS)

        self.get_logger().info(
            f"[hexa] meas={actual_topic}  est={est_topic}  cov={cov_topic}")

    def _sec(self, stamp):
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _cb_meas(self, msg):
        t = self._sec(msg.header.stamp)
        self.t_meas.append(t)
        for i in range(NUM_ROTORS):
            self.rpm_meas[i].append(float(msg.rpm[i]))

    def _cb_est(self, msg):
        t = self._sec(msg.header.stamp)
        self.t_est.append(t)
        for i in range(NUM_ROTORS):
            self.rpm_est[i].append(float(msg.rpm[i]))
            self.accel_est[i].append(float(msg.acceleration[i]))

    def _cb_cov(self, msg):
        t = self._sec(msg.header.stamp)
        self.t_cov.append(t)
        for i in range(NUM_ROTORS):
            self.sig_rpm[i].append(np.sqrt(max(msg.diag_cov[i], 0.0)))
            self.sig_acc[i].append(np.sqrt(max(msg.diag_cov[i + 6], 0.0)))


# ====================================================================
#  Plotting helpers
# ====================================================================
def _update_single_subplot(ax, t_est, y_est, t_meas, y_meas,
                           t_cov, sigma, color, fill_handle):
    """Return new fill_handle (or None)."""
    if fill_handle is not None:
        fill_handle.remove()
        fill_handle = None

    t_e = list(t_est)
    y_e = np.array(y_est)

    n_c = len(t_cov)
    n_e = len(t_e)

    if n_c > 1 and n_e > 1:
        n = min(n_e, n_c)
        sig = np.array(sigma)
        fill_handle = ax.fill_between(
            t_e[-n:],
            y_e[-n:] - 3.0 * sig[-n:],
            y_e[-n:] + 3.0 * sig[-n:],
            color=color, alpha=0.15)

    ax.relim()
    ax.autoscale_view()
    return fill_handle


# ====================================================================
#  Single-rotor main loop
# ====================================================================
def run_single(node):
    fig, (ax_rpm, ax_acc) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("Single Rotor — RTS Smoother", fontsize=14)

    ln_meas, = ax_rpm.plot([], [], "o", color="gray", ms=2, alpha=0.5,
                           label="Measured")
    ln_est,  = ax_rpm.plot([], [], "-", color="tab:blue", lw=1.5,
                           label="Estimated")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.legend(loc="upper left", fontsize=9)

    ln_acc, = ax_acc.plot([], [], "-", color="tab:orange", lw=1.5,
                          label="Estimated")
    ax_acc.set_ylabel("Accel [RPM/s]")
    ax_acc.set_xlabel("Time [s]")
    ax_acc.legend(loc="upper left", fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fill = {"rpm": None, "acc": None}

    def update(_):
        rclpy.spin_once(node, timeout_sec=0.0)

        if node.t_meas:
            ln_meas.set_data(list(node.t_meas), list(node.rpm_meas))
        if node.t_est:
            ln_est.set_data(list(node.t_est), list(node.rpm_est))
            ln_acc.set_data(list(node.t_est), list(node.accel_est))

        fill["rpm"] = _update_single_subplot(
            ax_rpm, node.t_est, node.rpm_est,
            node.t_meas, node.rpm_meas,
            node.t_cov, node.sig_rpm, "tab:blue", fill["rpm"])

        fill["acc"] = _update_single_subplot(
            ax_acc, node.t_est, node.accel_est,
            None, None,
            node.t_cov, node.sig_acc, "tab:orange", fill["acc"])

        return ln_meas, ln_est, ln_acc

    _ = FuncAnimation(fig, update, interval=50,
                      blit=False, cache_frame_data=False)
    plt.show()


# ====================================================================
#  Hexa-rotor main loop  (3 rows x 2 cols = 6 RPM subplots)
# ====================================================================
def run_hexa(node):
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    fig.suptitle("Hexacopter Rotors — RTS Smoother", fontsize=14)
    axes_flat = axes.flatten()  # [0..5]

    lines_meas = []
    lines_est  = []
    fills = [None] * NUM_ROTORS

    for i, ax in enumerate(axes_flat):
        lm, = ax.plot([], [], "o", color="gray", ms=1.5, alpha=0.4)
        le, = ax.plot([], [], "-", color=COLORS[i], lw=1.3)
        ax.set_ylabel(f"R{i} RPM", fontsize=9)
        ax.tick_params(labelsize=8)
        lines_meas.append(lm)
        lines_est.append(le)

    axes_flat[-2].set_xlabel("Time [s]", fontsize=9)
    axes_flat[-1].set_xlabel("Time [s]", fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    def update(_):
        rclpy.spin_once(node, timeout_sec=0.0)

        t_m = list(node.t_meas)
        t_e = list(node.t_est)

        for i in range(NUM_ROTORS):
            if t_m:
                lines_meas[i].set_data(t_m, list(node.rpm_meas[i]))
            if t_e:
                lines_est[i].set_data(t_e, list(node.rpm_est[i]))

            fills[i] = _update_single_subplot(
                axes_flat[i], node.t_est, node.rpm_est[i],
                node.t_meas, node.rpm_meas[i],
                node.t_cov, node.sig_rpm[i],
                COLORS[i], fills[i])

        return lines_meas + lines_est

    _ = FuncAnimation(fig, update, interval=50,
                      blit=False, cache_frame_data=False)
    plt.show()


# ====================================================================
#  Entry point
# ====================================================================
def main():
    rclpy.init()

    # Temporary node just to read the 'mode' parameter
    tmp = Node("_plotter_mode_reader")
    tmp.declare_parameter("mode", "single")
    mode = tmp.get_parameter("mode").value
    tmp.destroy_node()

    if mode == "hexa":
        node = HexaRotorPlotterNode()
        run_hexa(node)
    else:
        node = SingleRotorPlotterNode()
        run_single(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
