import time
from functools import partial
from typing import TYPE_CHECKING

from ardupilot_msgs.srv import Takeoff

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class TakingOffHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if not node.request_in_progress:
            self.call_takeoff(node)

    def call_takeoff(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True
        request = Takeoff.Request()
        request.alt = float(node.TAKEOFF_ALTITUDE)
        future = node.takeoff_client.call_async(request)
        future.add_done_callback(partial(self.takeoff_callback, node))
        node.get_logger().info(
            f"Requesting takeoff to {node.TAKEOFF_ALTITUDE:.1f} m"
        )

    def takeoff_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Takeoff service failed: {error}")
            return

        if not response.status:
            node.fail("ArduPilot rejected the takeoff request")
            return

        node.get_logger().info("Takeoff command accepted")
        node.get_logger().info(
            f"Hovering for {node.HOVER_DURATION:.1f} seconds"
        )
        node.hover_started_at = time.monotonic()
        node.state = FlightState.HOVERING
