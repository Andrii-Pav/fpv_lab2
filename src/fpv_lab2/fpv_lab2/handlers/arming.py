from functools import partial
from typing import TYPE_CHECKING

from ardupilot_msgs.srv import ArmMotors

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class ArmingHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if not node.request_in_progress:
            self.call_arm(node)

    def call_arm(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True
        request = ArmMotors.Request()
        request.arm = True
        future = node.arm_client.call_async(request)
        future.add_done_callback(partial(self.arm_callback, node))
        node.get_logger().info("Requesting motor arming")

    def arm_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Arming service failed: {error}")
            return

        if not response.result:
            node.fail("ArduPilot rejected the arming request")
            return

        node.get_logger().info("Motors armed")
        node.state = FlightState.TAKING_OFF
