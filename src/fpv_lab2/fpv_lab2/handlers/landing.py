from functools import partial
from typing import TYPE_CHECKING

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class LandingHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if not node.request_in_progress:
            node.call_mode_switch(
                node.LAND_MODE,
                partial(self.land_mode_callback, node),
            )

    def land_mode_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            node.fail(f"LAND mode request failed: {error}")
            return

        if not response.status or response.curr_mode != node.LAND_MODE:
            node.fail(
                f"Could not enable LAND mode, current mode: "
                f"{response.curr_mode}"
            )
            return

        node.get_logger().info("LAND mode enabled")
        node.state = FlightState.WAITING_FOR_LANDING
