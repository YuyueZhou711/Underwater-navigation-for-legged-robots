#!/usr/bin/env python3

import argparse
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


class SingleLegLifter(Node):
    def __init__(self, args):
        super().__init__("single_leg_lifter")
        self.args = args
        self.last_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states_stonefish", self._joint_state_cb, 10
        )
        self.joint_cmd_pub = self.create_publisher(
            JointState, "/joint_command_stonefish", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ft_topic = f"/silver2/ft/L{args.leg_id}"
        self.ft_echo_process = None
        self.last_commanded_positions = None
        self.stop_request_event = threading.Event()

        self.leg_joint_names = [
            f"silver2/Joint_L{args.leg_id}_Coxa",
            f"silver2/Joint_L{args.leg_id}_Femur",
            f"silver2/Joint_L{args.leg_id}_Tibia",
        ]

    def _joint_state_cb(self, msg):
        self.last_joint_state = msg

    def wait_for_joint_state(self, timeout_s):
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_joint_state is not None:
                return True
        return False

    def stop_gait_controller(self):
        twist = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

    def _extract_leg_positions(self):
        name_to_pos = dict(zip(self.last_joint_state.name, self.last_joint_state.position))
        current = []
        for joint_name in self.leg_joint_names:
            if joint_name not in name_to_pos:
                raise KeyError(f"Joint {joint_name} not found in /joint_states_stonefish")
            current.append(name_to_pos[joint_name])
        return current

    def _start_ft_echo_stream(self):
        cmd = ["ros2", "topic", "echo", self.ft_topic, "geometry_msgs/msg/WrenchStamped"]
        self.get_logger().info("Streaming FT at ~10Hz: %s" % " ".join(cmd))
        try:
            self.ft_echo_process = subprocess.Popen(cmd)
        except Exception as exc:
            self.ft_echo_process = None
            self.get_logger().error(f"Failed to start FT stream on {self.ft_topic}: {exc}")

    def _echo_ft_once_static(self):
        cmd = [
            "ros2",
            "topic",
            "echo",
            "--once",
            "--timeout",
            "3",
            self.ft_topic,
            "geometry_msgs/msg/WrenchStamped",
        ]
        self.get_logger().info("Static FT sample before lift: %s" % " ".join(cmd))
        try:
            result = subprocess.run(cmd, check=False)
            if result.returncode != 0:
                self.get_logger().warn(
                    f"Static FT sample returned code {result.returncode}, continue anyway."
                )
        except Exception as exc:
            self.get_logger().warn(f"Failed to get static FT sample on {self.ft_topic}: {exc}")

    def _stop_ft_echo_stream(self):
        if self.ft_echo_process is None:
            return
        if self.ft_echo_process.poll() is None:
            self.ft_echo_process.terminate()
            try:
                self.ft_echo_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.ft_echo_process.kill()
        self.ft_echo_process = None

    def _publish_leg_position_once(self, positions):
        msg = JointState()
        msg.name = self.leg_joint_names
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        self.last_commanded_positions = list(positions)
        self.joint_cmd_pub.publish(msg)

    def _publish_leg_position_for_duration(self, positions, duration_s, rate_hz, check_ok=True):
        dt = 1.0 / max(rate_hz, 1.0)

        if duration_s <= 0.0:
            while (rclpy.ok() if check_ok else True):
                self._publish_leg_position_once(positions)
                time.sleep(dt)
            return

        steps = max(1, int(duration_s * rate_hz))
        for _ in range(steps):
            if check_ok and not rclpy.ok():
                break
            self._publish_leg_position_once(positions)
            time.sleep(dt)

    def _smooth_move(self, start_positions, target_positions, move_time_s, rate_hz, check_ok=True):
        dt = 1.0 / max(rate_hz, 1.0)
        steps = max(1, int(move_time_s * rate_hz))
        for i in range(steps):
            if check_ok and not rclpy.ok():
                break
            alpha = float(i + 1) / float(steps)
            cmd = [
                start_positions[j] + alpha * (target_positions[j] - start_positions[j])
                for j in range(3)
            ]
            self._publish_leg_position_once(cmd)
            time.sleep(dt)

    def _restore_leg_position(self, original_positions, rate_hz):
        self.get_logger().info(
            "Restoring leg L%d to original position."
            % self.args.leg_id
        )
        start = self.last_commanded_positions
        if start is None:
            start = original_positions
        # Ignore rclpy.ok() here, so restore still runs after Ctrl+C.
        self._smooth_move(start, original_positions, 2.0, rate_hz, check_ok=False)
        self._publish_leg_position_for_duration(
            original_positions, 0.5, rate_hz, check_ok=False
        )

    def _wait_for_stop_command(self):
        while not self.stop_request_event.is_set():
            try:
                cmd = input().strip().lower()
            except EOFError:
                self.stop_request_event.set()
                return
            if cmd == "s":
                self.stop_request_event.set()
                return
            self.get_logger().info("Type 's' then Enter to restore and exit.")

    def run(self):
        if self.args.stop_cmd_vel:
            self.stop_gait_controller()

        if not self.wait_for_joint_state(self.args.state_timeout):
            raise RuntimeError("Timeout waiting for /joint_states_stonefish")

        coxa, femur, tibia = self._extract_leg_positions()
        target = [
            coxa + self.args.coxa_delta,
            femur + self.args.femur_delta,
            tibia + self.args.tibia_delta,
        ]

        self.get_logger().info(
            "Leg L%d current [%.3f, %.3f, %.3f] -> target [%.3f, %.3f, %.3f]"
            % (self.args.leg_id, coxa, femur, tibia, target[0], target[1], target[2])
        )

        original_positions = [coxa, femur, tibia]

        rate_hz = max(self.args.rate, 1.0)
        duration_s = self.args.duration
        hold_forever = duration_s <= 0.0
        if hold_forever:
            duration_s = 2.0
        if 0.0 < duration_s < 2.0:
            self.get_logger().info(
                "Duration %.2fs is short for FT observation. Extending to 2.00s."
                % duration_s
            )
            duration_s = 2.0

        interrupted = False
        should_restore = False
        try:
            # Print one baseline FT sample while leg is still static.
            self._echo_ft_once_static()

            self.get_logger().info(
                "Lifting leg over %.2fs (slower interpolated motion)." % duration_s
            )
            # Stream FT only during the lifting motion.
            self._start_ft_echo_stream()
            try:
                self._smooth_move(original_positions, target, duration_s, rate_hz)
            finally:
                self._stop_ft_echo_stream()

            if hold_forever:
                self.get_logger().info("Holding target. Type 's' then Enter to restore and exit.")
                listener = threading.Thread(target=self._wait_for_stop_command, daemon=True)
                listener.start()
                while not self.stop_request_event.is_set():
                    self._publish_leg_position_once(target)
                    time.sleep(1.0 / rate_hz)
                should_restore = True
        except KeyboardInterrupt:
            interrupted = True
            should_restore = True
        finally:
            if should_restore:
                self._restore_leg_position(original_positions, rate_hz)
            self._stop_ft_echo_stream()
            if interrupted:
                raise KeyboardInterrupt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Lift one selected leg by sending partial JointState commands."
    )
    parser.add_argument("--leg-id", type=int, required=True, choices=range(6))
    parser.add_argument(
        "--coxa-delta",
        type=float,
        default=0.0,
        help="Added to current coxa angle [rad].",
    )
    parser.add_argument(
        "--femur-delta",
        type=float,
        default=0.45,
        help="Added to current femur angle [rad].",
    )
    parser.add_argument(
        "--tibia-delta",
        type=float,
        default=-0.55,
        help="Added to current tibia angle [rad].",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Lift motion duration [s]. Values in (0,2) are forced to 2s. Use <=0 to lift in 2s then hold until you type 's'.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate [Hz].",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=3.0,
        help="Timeout waiting for /joint_states_stonefish [s].",
    )
    parser.add_argument(
        "--no-stop-cmd-vel",
        action="store_false",
        dest="stop_cmd_vel",
        help="Do not send zero /cmd_vel before lifting.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SingleLegLifter(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
