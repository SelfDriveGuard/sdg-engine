"""
Collection of TrafficEvents
"""

from enum import Enum


class ViolationEventType(Enum):

    """
    This enum represents different traffic events that occur during driving.
    """

    NORMAL_DRIVING = 0
    COLLISION_STATIC = 1
    COLLISION_VEHICLE = 2
    COLLISION_PEDESTRIAN = 3
    ROUTE_DEVIATION = 4
    ROUTE_COMPLETION = 5
    ROUTE_COMPLETED = 6
    TRAFFIC_LIGHT_INFRACTION = 7
    WRONG_WAY_INFRACTION = 8
    ON_SIDEWALK_INFRACTION = 9
    STOP_INFRACTION = 10
    OUTSIDE_LANE_INFRACTION = 11
    OUTSIDE_ROUTE_LANES_INFRACTION = 12
    VEHICLE_BLOCKED = 13
    SPEED_LIMIT_EXCEEDED = 14
    LOW_AVERAGE_VELOCITY = 15
    LANE_INVASION = 16
    OFF_ROAD = 17


class ViolationEvent(object):

    """
    TrafficEvent definition
    """

    def __init__(self, event_type, location, message=None):
        """
        Initialize object

        :param event_type: TrafficEventType defining the type of traffic event
        :param message: optional message to inform users of the event
        :param dictionary: optional dictionary with arbitrary keys and values
        """
        self._type = event_type
        self.location = location
        self._message = message

    def __str__(self):
        return f'ViolationEvent[{str(self._type)}] \
            occurred at Location[{str(self.location.x)},{str(self.location.y)},{str(self.location.z)}] \
            with message[{str(self._message)}]'

    def get_message(self):
        """
        @return message
        """
        if self._message:
            return self._message

        return ""

    def set_message(self, message):
        """
        Set message
        """
        self._message = message

