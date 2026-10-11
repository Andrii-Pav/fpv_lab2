from functools import partial
from typing import TYPE_CHECKING

from std_srvs.srv import Trigger

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class PrearmCheckHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if not node.request_in_progress:
            self.call_prearm_check(node)

    def call_prearm_check(self, node: "FlightTestNode") -> None:
        node.request_in_progress = True
        request = Trigger.Request()
        future = node.prearm_client.call_async(request)
        future.add_done_callback(partial(self.prearm_callback, node))
        node.get_logger().info("Running pre-arm check")

    def prearm_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Pre-arm service failed: {error}")
            return

        if not response.success:
            node.fail(f"Vehicle is not armable: {response.message}")
            return

        node.get_logger().info(f"Pre-arm check passed: {response.message}")
        node.state = FlightState.SETTING_GUIDED
