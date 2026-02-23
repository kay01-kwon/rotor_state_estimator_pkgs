#!/usr/bin/env python3
"""
Real-time plotter for rotor state estimation with 3-sigma bounds.

Supports two modes via the 'mode' parameter:
  - "single" : one rotor  (SingleActualRpm + SingleRotorCov)
  - "hexa"   : six rotors (HexaActualRpm + RotorCov)  — 6x2 grid
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
def _update_fill(ax, t_est, y_est, t_cov, sigma, color, fill_handle):
    """Redraw 3-sigma band. Return new fill_handle (or None)."""
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
    ax.autoscale_view(scalex=False, scaley=True)
    return fill_handle


def _set_xlim_from_deques(axes_list, *time_deques):
    """Set shared x-limits from the oldest/newest entries across time deques."""
    t_min = None
    t_max = None
    for td in time_deques:
        if len(td) > 0:
            lo = td[0]
            hi = td[-1]
            if t_min is None or lo < t_min:
                t_min = lo
            if t_max is None or hi > t_max:
                t_max = hi

    if t_min is not None and t_max is not None:
        margin = max((t_max - t_min) * 0.02, 0.05)
        for ax in axes_list:
            ax.set_xlim(t_min - margin, t_max + margin)


# ====================================================================
#  Single-rotor main loop
# ====================================================================
def run_single(node):
    fig, (ax_rpm, ax_acc) = plt.subplots(
        2, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("Single Rotor — RTS Smoother", fontsize=14)

    ln_meas, = ax_rpm.plot([], [], "o", color="gray", ms=5, alpha=0.6,
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

        fill["rpm"] = _update_fill(
            ax_rpm, node.t_est, node.rpm_est,
            node.t_cov, node.sig_rpm, "tab:blue", fill["rpm"])

        fill["acc"] = _update_fill(
            ax_acc, node.t_est, node.accel_est,
            node.t_cov, node.sig_acc, "tab:orange", fill["acc"])

        _set_xlim_from_deques([ax_rpm, ax_acc],
                              node.t_meas, node.t_est, node.t_cov)

        return ln_meas, ln_est, ln_acc

    _ = FuncAnimation(fig, update, interval=50,
                      blit=False, cache_frame_data=False)
    plt.show()


# ====================================================================
#  Hexa-rotor main loop
#  6 rows x 2 cols:  left = RPM + 3sigma,  right = Accel + 3sigma
# ====================================================================
def run_hexa(node):
    fig, axes = plt.subplots(
        NUM_ROTORS, 2, figsize=(16, 14), sharex=True)
    fig.suptitle("Hexacopter Rotors — RTS Smoother", fontsize=14)

    ln_meas_rpm = []
    ln_est_rpm  = []
    ln_est_acc  = []
    fill_rpm    = [None] * NUM_ROTORS
    fill_acc    = [None] * NUM_ROTORS

    for i in range(NUM_ROTORS):
        ax_r = axes[i, 0]  # RPM column
        ax_a = axes[i, 1]  # Accel column

        lm, = ax_r.plot([], [], "o", color="gray", ms=4, alpha=0.5)
        le, = ax_r.plot([], [], "-", color=COLORS[i], lw=1.3)
        la, = ax_a.plot([], [], "-", color=COLORS[i], lw=1.3)

        ax_r.set_ylabel(f"R{i} RPM", fontsize=8)
        ax_a.set_ylabel(f"R{i} Acc", fontsize=8)
        ax_r.tick_params(labelsize=7)
        ax_a.tick_params(labelsize=7)

        ln_meas_rpm.append(lm)
        ln_est_rpm.append(le)
        ln_est_acc.append(la)

    axes[0, 0].set_title("RPM  (meas + est + 3σ)", fontsize=10)
    axes[0, 1].set_title("Acceleration  (est + 3σ)", fontsize=10)
    axes[-1, 0].set_xlabel("Time [s]", fontsize=9)
    axes[-1, 1].set_xlabel("Time [s]", fontsize=9)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    def update(_):
        rclpy.spin_once(node, timeout_sec=0.0)

        t_m = list(node.t_meas)
        t_e = list(node.t_est)

        for i in range(NUM_ROTORS):
            if t_m:
                ln_meas_rpm[i].set_data(t_m, list(node.rpm_meas[i]))
            if t_e:
                ln_est_rpm[i].set_data(t_e, list(node.rpm_est[i]))
                ln_est_acc[i].set_data(t_e, list(node.accel_est[i]))

            fill_rpm[i] = _update_fill(
                axes[i, 0], node.t_est, node.rpm_est[i],
                node.t_cov, node.sig_rpm[i],
                COLORS[i], fill_rpm[i])

            fill_acc[i] = _update_fill(
                axes[i, 1], node.t_est, node.accel_est[i],
                node.t_cov, node.sig_acc[i],
                COLORS[i], fill_acc[i])

        _set_xlim_from_deques(axes.flatten().tolist(),
                              node.t_meas, node.t_est, node.t_cov)

        return ln_meas_rpm + ln_est_rpm + ln_est_acc

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
