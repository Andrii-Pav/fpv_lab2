from typing import TYPE_CHECKING

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class WaitingForSystemHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if node.status is None:
            return

        if not node.services_ready():
            return

        node.get_logger().info("ArduPilot and all services are available")
        node.state = FlightState.PREARM_CHECK
