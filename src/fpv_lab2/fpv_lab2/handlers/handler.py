from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class FlightStateHandler(ABC):
    @abstractmethod
    def handle(self, node: "FlightTestNode") -> None:
        pass
