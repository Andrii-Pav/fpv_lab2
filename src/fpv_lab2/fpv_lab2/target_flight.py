#!/usr/bin/env python3
"""Політ дрона до заданої точки з подальшою автоматичною посадкою.

Варіант 14: dx = 6.4 м, dy = 0.2 м.
    x_target = x0 - dx
    y_target = y0 - dy
де x0, y0 - фактичне початкове положення дрона.
"""
import math
import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

from .flight_test_node import FlightTestNode


class TargetState(Enum):
    WAITING_FOR_SYSTEM = auto()
    PREARM_CHECK = auto()
    SETTING_GUIDED = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    CLIMBING = auto()
    STABILIZING = auto()
    MOVING_TO_TARGET = auto()
    STOPPING = auto()
    HOLDING = auto()
    LANDING = auto()
    WAITING_FOR_LANDING = auto()
    FINISHED = auto()
    ERROR = auto()


class TargetNavigator:
    """Відстань, напрямок і швидкість руху до цілі (пропорційне керування)."""

    def __init__(self, max_speed, min_speed, gain):
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.gain = gain

    @staticmethod
    def error_vector(x, y, target_x, target_y):
        error_x = target_x - x
        error_y = target_y - y
        return error_x, error_y, math.hypot(error_x, error_y)

    def velocity(self, error_x, error_y, distance):
        """Швидкість пропорційна відстані: біля цілі дрон сповільнюється."""
        if distance < 1e-6:
            return 0.0, 0.0
        speed = min(self.max_speed, max(self.min_speed, self.gain * distance))
        return speed * error_x / distance, speed * error_y / distance


class TargetFlightNode(FlightTestNode):
    # Індивідуальне завдання (варіант 14)
    DELTA_X = 6.4
    DELTA_Y = 0.2

    # Параметри алгоритму
    TARGET_TOLERANCE = 0.2      # м, поріг досягнення цілі
    ACCEPTABLE_ERROR = 0.3      # м, допустима похибка за методичкою
    MAX_SPEED = 1.0             # м/с
    MIN_SPEED = 0.1             # м/с
    GAIN = 0.6                  # 1/с, коефіцієнт пропорційного керування
    CLIMB_RATIO = 0.9           # частка висоти зльоту для переходу далі
    CLIMB_TIMEOUT = 30.0        # с
    STABILIZE_TIME = 3.0        # с зависання перед рухом
    SETTLE_TIME = 3.0           # с на гасіння швидкості біля цілі
    HOLD_TIME = 10.0            # с зависання над ціллю (час для скриншота)
    MOVE_TIMEOUT = 120.0        # с

    def __init__(self):
        super().__init__()

        self.state = TargetState.WAITING_FOR_SYSTEM
        self.pose = None
        self.start_x = None
        self.start_y = None
        self.target_x = None
        self.target_y = None
        self.phase_started_at = time.monotonic()
        self.last_log_at = 0.0
        self.navigator = TargetNavigator(
            self.MAX_SPEED, self.MIN_SPEED, self.GAIN
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            "/ap/v1/pose/filtered",
            self.pose_callback,
            qos_profile_sensor_data,
        )
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            "/ap/v1/cmd_vel",
            10,
        )

        # Команди швидкості потрібно слати часто, тому цикл 10 Гц
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(0.1, self.control_loop)

        self._steps = {
            TargetState.WAITING_FOR_SYSTEM: self.step_waiting,
            TargetState.PREARM_CHECK: self.step_prearm,
            TargetState.SETTING_GUIDED: self.step_guided,
            TargetState.ARMING: self.step_arming,
            TargetState.TAKING_OFF: self.step_takeoff,
            TargetState.CLIMBING: self.step_climbing,
            TargetState.STABILIZING: self.step_stabilizing,
            TargetState.MOVING_TO_TARGET: self.step_moving,
            TargetState.STOPPING: self.step_stopping,
            TargetState.HOLDING: self.step_holding,
            TargetState.LANDING: self.step_landing,
            TargetState.WAITING_FOR_LANDING: self.step_waiting_for_landing,
        }

        self.get_logger().info(
            f"Target flight: dx={self.DELTA_X}, dy={self.DELTA_Y}, "
            f"tolerance={self.TARGET_TOLERANCE} m"
        )

    # ---------- допоміжні методи ----------

    def pose_callback(self, message):
        self.pose = message

    def position(self):
        point = self.pose.pose.position
        return point.x, point.y, point.z

    def gazebo_position(self):
        if self.odometry is None:
            return None
        point = self.odometry.pose.pose.position
        return point.x, point.y

    def set_state(self, new_state):
        self.state = new_state
        self.phase_started_at = time.monotonic()

    def phase_elapsed(self):
        return time.monotonic() - self.phase_started_at

    def log_every(self, period, text):
        now = time.monotonic()
        if now - self.last_log_at >= period:
            self.get_logger().info(text)
            self.last_log_at = now

    def publish_velocity(self, velocity_x, velocity_y):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = float(velocity_x)
        message.twist.linear.y = float(velocity_y)
        message.twist.linear.z = 0.0
        self.velocity_publisher.publish(message)

    def fail(self, message):
        self.get_logger().error(message)
        self.publish_velocity(0.0, 0.0)
        self.state = TargetState.ERROR

    def call_service(self, client, request, name, on_response):
        self.request_in_progress = True
        future = client.call_async(request)
        future.add_done_callback(
            lambda done: self.service_done(done, name, on_response)
        )
        self.get_logger().info(f"Requesting: {name}")

    def service_done(self, future, name, on_response):
        self.request_in_progress = False
        try:
            response = future.result()
        except Exception as error:
            self.fail(f"{name} failed: {error}")
            return
        on_response(response)

    def control_loop(self):
        step = self._steps.get(self.state)
        if step is not None:
            step()

    # ---------- етапи польоту ----------

    def step_waiting(self):
        if self.status is None or self.pose is None:
            return
        if not self.services_ready():
            return

        self.start_x, self.start_y, _ = self.position()
        self.target_x = self.start_x - self.DELTA_X
        self.target_y = self.start_y - self.DELTA_Y

        self.get_logger().info("System is ready")
        self.get_logger().info(
            f"Start position:  x={self.start_x:.3f}, y={self.start_y:.3f}"
        )
        self.get_logger().info(
            f"Target position: x={self.target_x:.3f}, y={self.target_y:.3f}"
        )
        gazebo = self.gazebo_position()
        if gazebo is not None:
            self.get_logger().info(
                f"Gazebo odometry at start: x={gazebo[0]:.3f}, "
                f"y={gazebo[1]:.3f}"
            )
        self.set_state(TargetState.PREARM_CHECK)

    def step_prearm(self):
        if self.request_in_progress:
            return
        self.call_service(
            self.prearm_client, Trigger.Request(), "pre-arm check",
            self.on_prearm,
        )

    def on_prearm(self, response):
        if not response.success:
            self.fail(f"Vehicle is not armable: {response.message}")
            return
        self.get_logger().info(f"Pre-arm check passed: {response.message}")
        self.set_state(TargetState.SETTING_GUIDED)

    def step_guided(self):
        if self.request_in_progress:
            return
        request = ModeSwitch.Request()
        request.mode = self.GUIDED_MODE
        self.call_service(
            self.mode_client, request, "GUIDED mode",
            lambda r: self.on_mode(
                r, self.GUIDED_MODE, "GUIDED", TargetState.ARMING
            ),
        )

    def on_mode(self, response, mode, name, next_state):
        if not response.status or response.curr_mode != mode:
            self.fail(
                f"Could not enable {name} mode, "
                f"current mode: {response.curr_mode}"
            )
            return
        self.get_logger().info(f"{name} mode enabled")
        self.set_state(next_state)

    def step_arming(self):
        if self.request_in_progress:
            return
        request = ArmMotors.Request()
        request.arm = True
        self.call_service(
            self.arm_client, request, "motor arming", self.on_arm
        )

    def on_arm(self, response):
        if not response.result:
            self.fail("ArduPilot rejected the arming request")
            return
        self.get_logger().info("Motors armed")
        self.set_state(TargetState.TAKING_OFF)

    def step_takeoff(self):
        if self.request_in_progress:
            return
        request = Takeoff.Request()
        request.alt = float(self.TAKEOFF_ALTITUDE)
        self.call_service(
            self.takeoff_client, request,
            f"takeoff to {self.TAKEOFF_ALTITUDE:.1f} m", self.on_takeoff,
        )

    def on_takeoff(self, response):
        if not response.status:
            self.fail("ArduPilot rejected the takeoff request")
            return
        self.get_logger().info("Takeoff command accepted")
        self.set_state(TargetState.CLIMBING)

    def step_climbing(self):
        altitude = self.position()[2]
        if altitude >= self.CLIMB_RATIO * self.TAKEOFF_ALTITUDE:
            self.get_logger().info(
                f"Takeoff altitude reached: {altitude:.2f} m, stabilizing"
            )
            self.set_state(TargetState.STABILIZING)
        elif self.phase_elapsed() > self.CLIMB_TIMEOUT:
            self.fail("Takeoff altitude was not reached in time")

    def step_stabilizing(self):
        if self.status is None or not self.status.armed:
            self.fail("Drone disarmed unexpectedly during flight")
            return
        if self.phase_elapsed() >= self.STABILIZE_TIME:
            self.get_logger().info("Hover stable, starting flight to target")
            self.set_state(TargetState.MOVING_TO_TARGET)

    def step_moving(self):
        x, y, _ = self.position()
        error_x, error_y, distance = self.navigator.error_vector(
            x, y, self.target_x, self.target_y
        )

        if distance <= self.TARGET_TOLERANCE:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info(
                f"Target reached (distance {distance:.3f} m), stopping"
            )
            self.set_state(TargetState.STOPPING)
            return

        if self.phase_elapsed() > self.MOVE_TIMEOUT:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().warning("Move timeout, stopping")
            self.set_state(TargetState.STOPPING)
            return

        velocity_x, velocity_y = self.navigator.velocity(
            error_x, error_y, distance
        )
        self.publish_velocity(velocity_x, velocity_y)
        self.log_every(
            1.0,
            f"Position: x={x:.2f}, y={y:.2f}, distance={distance:.2f} m, "
            f"speed={math.hypot(velocity_x, velocity_y):.2f} m/s",
        )

    def step_stopping(self):
        self.publish_velocity(0.0, 0.0)
        if self.phase_elapsed() < self.SETTLE_TIME:
            return

        x, y, _ = self.position()
        error = math.hypot(x - self.target_x, y - self.target_y)

        self.get_logger().info("--- Flight result ---")
        self.get_logger().info(
            f"Start position:  x={self.start_x:.3f}, y={self.start_y:.3f}"
        )
        self.get_logger().info(
            f"Target position: x={self.target_x:.3f}, y={self.target_y:.3f}"
        )
        self.get_logger().info(
            f"Actual position before landing: x={x:.3f}, y={y:.3f}"
        )
        self.get_logger().info(f"Positioning error: {error:.3f} m")
        if error <= self.ACCEPTABLE_ERROR:
            self.get_logger().info("Target reached within tolerance")
        else:
            self.get_logger().warning("Error exceeds acceptable tolerance")

        gazebo = self.gazebo_position()
        if gazebo is not None:
            self.get_logger().info(
                f"Gazebo odometry now: x={gazebo[0]:.3f}, y={gazebo[1]:.3f}"
            )
        self.get_logger().info(
            f"Hovering {self.HOLD_TIME:.0f} s above target (screenshot)"
        )
        self.set_state(TargetState.HOLDING)

    def step_holding(self):
        self.publish_velocity(0.0, 0.0)
        if self.phase_elapsed() >= self.HOLD_TIME:
            self.get_logger().info("Starting landing")
            self.set_state(TargetState.LANDING)

    def step_landing(self):
        if self.request_in_progress:
            return
        request = ModeSwitch.Request()
        request.mode = self.LAND_MODE
        self.call_service(
            self.mode_client, request, "LAND mode",
            lambda r: self.on_mode(
                r, self.LAND_MODE, "LAND", TargetState.WAITING_FOR_LANDING
            ),
        )

    def step_waiting_for_landing(self):
        if self.status is None or self.status.flying:
            return
        self.get_logger().info("Landing completed")
        if self.pose is not None:
            self.get_logger().info(
                f"Final altitude: {self.position()[2]:.2f} m"
            )
        self.set_state(TargetState.FINISHED)
        self.get_logger().info("Target flight completed successfully")


def main(args=None):
    rclpy.init(args=args)
    node = TargetFlightNode()

    try:
        while (
            rclpy.ok()
            and node.state != TargetState.FINISHED
            and node.state != TargetState.ERROR
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
