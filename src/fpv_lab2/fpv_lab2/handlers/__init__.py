from ..flight_state import FlightState
from .arming import ArmingHandler
from .handler import FlightStateHandler
from .hovering import HoveringHandler
from .landing import LandingHandler
from .prearm_check import PrearmCheckHandler
from .setting_guided import SettingGuidedHandler
from .taking_off import TakingOffHandler
from .waiting_for_landing import WaitingForLandingHandler
from .waiting_for_system import WaitingForSystemHandler

HANDLERS: dict[FlightState, FlightStateHandler] = {
    FlightState.WAITING_FOR_SYSTEM: WaitingForSystemHandler(),
    FlightState.PREARM_CHECK: PrearmCheckHandler(),
    FlightState.SETTING_GUIDED: SettingGuidedHandler(),
    FlightState.ARMING: ArmingHandler(),
    FlightState.TAKING_OFF: TakingOffHandler(),
    FlightState.HOVERING: HoveringHandler(),
    FlightState.LANDING: LandingHandler(),
    FlightState.WAITING_FOR_LANDING: WaitingForLandingHandler(),
}
