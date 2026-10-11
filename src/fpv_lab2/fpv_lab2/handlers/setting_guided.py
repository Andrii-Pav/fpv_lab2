from functools import partial
from typing import TYPE_CHECKING

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class SettingGuidedHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if not node.request_in_progress:
            node.call_mode_switch(
                node.GUIDED_MODE,
                partial(self.guided_mode_callback, node),
            )

    def guided_mode_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            node.fail(f"GUIDED mode request failed: {error}")
            return

        if not response.status or response.curr_mode != node.GUIDED_MODE:
            node.fail(
                f"Could not enable GUIDED mode, current mode: "
                f"{response.curr_mode}"
            )
            return

        node.get_logger().info("GUIDED mode enabled")
        node.state = FlightState.ARMING
