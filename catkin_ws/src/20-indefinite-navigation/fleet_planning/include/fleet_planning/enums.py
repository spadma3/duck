from enum import Enum, IntEnum

class TaxiState(IntEnum):
    GOING_TO_CUSTOMER = 0
    WITH_CUSTOMER = 1
    IDLE = 2


class Instruction(Enum):
    LEFT = 'l'
    RIGHT = 'r'
    STRAIGHT = 's'


class FleetPlanningStrategy(Enum): # for future expansion
    DEACTIVATED = 0
    CLOSEST_DUCKIEBOT = 1