from enum import IntEnum

class MSG_TYPE(IntEnum):
    INFO = 0
    ERROR = 1
    GPS = 2
    EVENT_GENERATED = 3
    EVENT_RECEIVED = 4
    VEHICLE_STOPPED = 5
    VEHICLE_RELEASED = 6
    CONFIGURED = 7
