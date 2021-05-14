"""
This module provides all atomic evaluation criteria required to analyze if a
scenario was completed successfully or failed.

Criteria should run continuously to monitor the state of a single actor, multiple
actors or environmental parameters. Hence, a termination is not required.

The atomic criteria are implemented with py_trees.
"""

import weakref
import math
import numpy as np

import carla
from tabulate import tabulate
from src.tools.traffic_events import ViolationEvent, ViolationEventType
from src.tools.utils import RepeatedTimer
import shapely


def calculate_velocity(actor):
    """
    Method to calculate the velocity of a actor
    """
    velocity_squared = actor.get_velocity().x**2
    velocity_squared += actor.get_velocity().y**2
    return math.sqrt(velocity_squared)


class CriteriaManager(object):

    """
    This is the statistics manager for the CARLA leaderboard.
    It gathers data at runtime via the scenario evaluation criteria.
    """
    # data collection
    _actor_velocity_map = dict()
    _actor_location_map = dict()
    _actor_transform_map = dict()
    _carla_adapter = None

    def __init__(self, carla_adapter, interval=1):
        self._interval = interval
        CriteriaManager._carla_adapter = carla_adapter
        self._registry_criteria_list = []
        self.criteria_thread = None

    def get_check_interval(self):
        """
        returns the interval of evalutaion and data collection
        """
        return self._interval

    def registry_criteria(self, criteria):
        """
        register a single criterion
        """
        self._registry_criteria_list.append(criteria)

    def registry_criterias(self, criterias):
        """
        register a list of criteria
        """
        self._registry_criteria_list.extend(criterias)

    def clear_criteria(self):
        """
        clear all registered criteria
        """
        self._registry_criteria_list.clear()

    @staticmethod
    def get_velocity(actor):
        """
        returns the absolute velocity for the given actor
        """
        for key in CriteriaManager._actor_velocity_map:
            if key.id == actor.id:
                if len(CriteriaManager._actor_velocity_map[key]) > 0:
                    return CriteriaManager._actor_velocity_map[key][-1]

        print('{}.get_velocity: {} not found!' .format(__name__, actor))
        return 0.0

    @staticmethod
    def get_location(actor):
        """
        returns the location for the given actor
        """

        for key in CriteriaManager._actor_location_map:

            if key.id == actor.id:

                if len(CriteriaManager._actor_location_map[key]) > 0:
                    return CriteriaManager._actor_location_map[key][-1]

        print('{}.get_location: {} not found!' .format(__name__, actor))
        return None

    @staticmethod
    def get_transform(actor):
        """
        returns the transform for the given actor
        """
        for key in CriteriaManager._actor_transform_map:
            if key.id == actor.id:
                if len(CriteriaManager._actor_transform_map[key]) > 0:
                    return CriteriaManager._actor_transform_map[key][-1]

        print('{}.get_transform: {} not found!' .format(__name__, actor))
        return None

    def start_to_evaluate(self):
        self.criteria_thread = RepeatedTimer(
            self._interval, self.update)  # auto starts

    def stop(self):
        if self.criteria_thread:
            self.criteria_thread.stop()

    def update(self):
        """
        collect data from the carla simulator and update the evaluation results
        """
        # data collection
        snapshot = CriteriaManager._carla_adapter.world.get_snapshot()
        timestamp = snapshot.timestamp
        for actor in CriteriaManager._carla_adapter.world.get_actors():
            if actor in CriteriaManager._actor_velocity_map:
                CriteriaManager._actor_velocity_map[actor].append({
                    'elapsed_seconds': timestamp.elapsed_seconds,
                    'value': calculate_velocity(actor)
                })
                CriteriaManager._actor_location_map[actor].append({
                    'elapsed_seconds': timestamp.elapsed_seconds,
                    'value': actor.get_location()
                })
                CriteriaManager._actor_transform_map[actor].append({
                    'elapsed_seconds': timestamp.elapsed_seconds,
                    'value': actor.get_transform()
                })
            else:
                CriteriaManager._actor_velocity_map[actor] = [{
                    'elapsed_seconds': timestamp.elapsed_seconds,
                    'value': calculate_velocity(actor)
                }]
                CriteriaManager._actor_location_map[actor] = [{
                    'elapsed_seconds': timestamp.elapsed_seconds,
                    'value': actor.get_location()
                }]
                CriteriaManager._actor_transform_map[actor] = [{
                    'elapsed_seconds': timestamp.elapsed_seconds,
                    'value': actor.get_transform()
                }]

        # evaluation
        for criteria in self._registry_criteria_list:
            criteria.update()

    def compute_global_statistics(self):
        """
        returns the score of a single test case
        output the score sheet to the console
        """
        if self.criteria_thread:
            # Initialize the frontend msg
            frontend_msg = {
                'list': []
            }

            # Initialize the console tabulate
            header = ['Criterion', 'Result', 'Penalty']
            list_statistics = [header]

            # Compute penalty
            global_score = 100
            for criteria in self._registry_criteria_list:
                result = "FAILURE" if criteria.failure_count > 0 else "SUCCESS"
                if isinstance(criteria, CollisionTest):
                    actual_value = criteria.failure_count
                else:
                    actual_value = round(
                        criteria.failure_count/criteria.test_count, 2) if criteria.test_count > 0 else 0.0
                global_score -= actual_value          
                list_statistics.extend([[str(criteria), result, -actual_value]])
                frontend_msg['list'].append({
                    'criterion':str(criteria),
                    'result':result,
                    'penalty':-actual_value
                })

            # Complete the frontend msg
            frontend_msg['score'] = global_score

            # Complete the console tabulate
            output = "====== Result of Test ======"
            output += "\n"
            output += f">score:{global_score}\n"
            output += tabulate(list_statistics, tablefmt='fancy_grid')

            print(output)
            return frontend_msg

        print('the criteria_thread did not start!')
        return ''

    def get_global_event_report(self):
        """
        output the violation events and details to the console
        """
        if self.criteria_thread:
            for criteria in self._registry_criteria_list:
                print(f'====== Criterion type: {criteria} ======')
                for event in criteria.list_traffic_events:
                    print(event)
        else:
            print('the criteria_thread did not start!')

    def save_global_record(self, route_record, sensors, total_routes, endpoint):
        """
        save global record to specific file: endpoint
        """
        # data = fetch_dict(endpoint)
        # data = create_default_json_msg()
        # save_dict(endpoint, data)
        pass

    def clear_record(self, endpoint):
        """
        clean global record in specific file: endpoint
        """
        pass


class Criterion:

    """
    Base class for all criteria used to evaluate a scenario for success/failure

    Important parameters (PUBLIC):
    - name: Name of the criterion
    - expected_value_success:    Result in case of success
                                 (e.g. max_speed, zero collisions, ...)
    - expected_value_acceptable: Result that does not mean a failure,
                                 but is not good enough for a success
    - actual_value: Actual result after running the scenario
    - test_count: Used to record the total number of tests
    - failure_count: Used to record the number of test failures
    - optional: Indicates if a criterion is optional (not used for overall analysis)
    """

    def __init__(self,
                 name,
                 actor,
                 expected_value_success,
                 expected_value_acceptable=None,
                 optional=False):

        self.name = name
        self.actor = actor
        self.expected_value_success = expected_value_success
        self.expected_value_acceptable = expected_value_acceptable
        self.actual_value = 0
        self.optional = optional
        self.test_count = 0
        self.failure_count = 0
        self.list_traffic_events = []

    def __str__(self):
        return f'{self.name}'

    def update(self):

        self.test_count += 1


class MaxVelocityTest(Criterion):

    """
    This class contains an atomic test for maximum velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - max_velocity_allowed: maximum allowed velocity in km/h
    - max_velocity_accepted: If the actor's max velocity is more than this value (in km/h),
                             the test result is ACCEPTABLE
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self,
                 actor,
                 max_velocity_allowed=10,
                 max_velocity_accepted=20,
                 optional=False,
                 name="CheckMaximumVelocity"):
        """
        Setup actor and maximum allowed velovity
        """
        super(MaxVelocityTest, self).__init__(name, actor,
                                              max_velocity_allowed,
                                              max_velocity_accepted,
                                              optional)

    def update(self):
        """
        Check velocity
        """
        super(MaxVelocityTest, self).update()

        if self.actor is None:
            print('[warning]: no actor')
            return

        # [m/s] -> [km/h]
        vel = self.actor.get_velocity()
        velocity = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        self.actual_value = velocity

        if self.actual_value > self.expected_value_success and not self.optional:
            self.failure_count += 1
            event_type = ViolationEventType.SPEED_LIMIT_EXCEEDED
            location = self.actor.get_location()
            collision_event = ViolationEvent(
                event_type=event_type, location=location)
            self.list_traffic_events.append(collision_event)


class AverageVelocityTest(Criterion):

    """
    This class contains an atomic test for average velocity.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - avg_velocity_success: If the actor's average velocity is more than this value (in km/h),
                            the test result is SUCCESS
    - avg_velocity_acceptable: If the actor's average velocity is more than this value (in km/h),
                               the test result is ACCEPTABLE
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self,
                 actor,
                 avg_velocity_success,
                 avg_velocity_acceptable=None,
                 optional=False,
                 name="CheckAverageVelocity"):
        """
        Setup actor and average velovity expected
        """
        super(AverageVelocityTest, self).__init__(name, actor,
                                                  avg_velocity_success,
                                                  avg_velocity_acceptable,
                                                  optional)
        # 本次interval开始时的location
        self._last_location = None
        # 模拟开始至今的总路程
        self._distance = 0.0
        # 模拟开始至今的时间
        self._test_time = None

    def update(self):
        """
        Check velocity
        """
        super(AverageVelocityTest, self).update()

        location = self.actor.get_location()

        _last_location_dict = CriteriaManager.get_location(self.actor)

        self._distance += location.distance(_last_location_dict['value'])
        self._last_location = location

        # Counts the time offroad
        if self._test_time is None:
            self._test_time = _last_location_dict['elapsed_seconds']
        else:
            self.actual_value = 3.6 * self._distance / self._test_time

        if self.actual_value < self.expected_value_success:
            self.failure_count += 1
            event_type = ViolationEventType.LOW_AVERAGE_VELOCITY
            location = self.actor.get_location()
            collision_event = ViolationEvent(
                event_type=event_type, location=location)
            self.list_traffic_events.append(collision_event)


class CollisionTest(Criterion):

    """
    This class contains an atomic test for collisions.

    Args:
    - actor (carla.Actor): CARLA actor to be used for this test
    - other_actor (carla.Actor): only collisions with this actor will be registered
    - other_actor_type (str): only collisions with actors including this type_id will count.
        Additionally, the "miscellaneous" tag can also be used to include all static objects in the scene
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    # If closer than this distance, the collision is ignored
    MIN_AREA_OF_COLLISION = 3
    MAX_AREA_OF_COLLISION = 5       # If further than this distance, the area is forgotten
    MAX_ID_TIME = 5                 # Amount of time the last collision if is remembered

    def __init__(self, actor, other_actor=None, other_actor_type=None,
                 optional=False, name="CollisionTest"):
        """
        Construction with sensor setup
        """
        super(CollisionTest, self).__init__(name, actor, 0, None, optional)

        world = self.actor.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.collision')
        self._collision_sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=self.actor)
        self._collision_sensor.listen(
            lambda event: self._count_collisions(weakref.ref(self), event))
        CriteriaManager._carla_adapter.actor_list.append(
            self._collision_sensor)

        self.other_actor = other_actor
        self.other_actor_type = other_actor_type
        self.registered_collisions = []
        self.last_id = None
        self.collision_time = None

    def update(self):
        """
        Check collision count
        """
        super(CollisionTest, self).update()

        _last_location_dict = CriteriaManager.get_location(self.actor)

        actor_location = _last_location_dict['value']
        new_registered_collisions = []

        # Loops through all the previous registered collisions
        for collision_location in self.registered_collisions:

            # Get the distance to the collision point
            distance_vector = actor_location - collision_location
            distance = math.sqrt(
                math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

            # If far away from a previous collision, forget it
            if distance <= self.MAX_AREA_OF_COLLISION:
                new_registered_collisions.append(collision_location)

        self.registered_collisions = new_registered_collisions

        if self.last_id and _last_location_dict['elapsed_seconds'] - self.collision_time > self.MAX_ID_TIME:
            self.last_id = None

    @staticmethod
    def _count_collisions(weak_self, event):     # pylint: disable=too-many-return-statements
        """
        Callback to update collision count
        """
        self = weak_self()
        if not self:
            return

        _last_location_dict = CriteriaManager.get_location(self.actor)
        actor_location = _last_location_dict['value']

        # Ignore the current one if it is the same id as before
        if self.last_id == event.other_actor.id:
            return

        # Filter to only a specific actor
        if self.other_actor and self.other_actor.id != event.other_actor.id:
            return

        # Filter to only a specific type
        if self.other_actor_type:
            if self.other_actor_type == "miscellaneous":
                if "traffic" not in event.other_actor.type_id \
                        and "static" not in event.other_actor.type_id:
                    return
            else:
                if self.other_actor_type not in event.other_actor.type_id:
                    return

        # Ignore it if its too close to a previous collision (avoid micro collisions)
        for collision_location in self.registered_collisions:

            distance_vector = actor_location - collision_location
            distance = math.sqrt(
                math.pow(distance_vector.x, 2) + math.pow(distance_vector.y, 2))

            if distance <= self.MIN_AREA_OF_COLLISION:
                return

        if ('static' in event.other_actor.type_id or 'traffic' in event.other_actor.type_id) \
                and 'sidewalk' not in event.other_actor.type_id:
            actor_type = ViolationEventType.COLLISION_STATIC
        elif 'vehicle' in event.other_actor.type_id:
            actor_type = ViolationEventType.COLLISION_VEHICLE
        elif 'walker' in event.other_actor.type_id:
            actor_type = ViolationEventType.COLLISION_PEDESTRIAN
        else:
            return

        collision_event = ViolationEvent(
            event_type=actor_type, location=actor_location)
        collision_event.set_message(
            "Agent collided against object with type={} and id={} at (x={}, y={}, z={})".format(
                event.other_actor.type_id,
                event.other_actor.id,
                round(actor_location.x, 3),
                round(actor_location.y, 3),
                round(actor_location.z, 3)))

        self.collision_time = _last_location_dict['elapsed_seconds']

        self.registered_collisions.append(actor_location)
        self.list_traffic_events.append(collision_event)
        self.failure_count += 1

        # Number 0: static objects -> ignore it
        if event.other_actor.id != 0:
            self.last_id = event.other_actor.id


class ActorSpeedAboveThresholdTest(Criterion):

    """
    This test will fail if the actor has had its linear velocity lower than a specific value for
    a specific amount of time
    Important parameters:
    - actor: CARLA actor to be used for this test
    - speed_threshold: speed required
    - below_threshold_max_time: Maximum time (in seconds) the actor can remain under the speed threshold
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """

    def __init__(self, actor, speed_threshold, below_threshold_max_time,
                 name="AgentBlockedTest"):
        """
        Class constructor.
        """
        super(ActorSpeedAboveThresholdTest, self).__init__(name, actor, 0)
        self._actor = actor
        self._speed_threshold = speed_threshold
        self._below_threshold_max_time = below_threshold_max_time
        self._time_last_valid_state = None

    def update(self):
        """
        Check if the actor speed is above the speed_threshold
        """
        _last_velocity_dict = CriteriaManager.get_velocity(self.actor)
        linear_speed = _last_velocity_dict['value']

        if linear_speed is not None:
            if linear_speed < self._speed_threshold and self._time_last_valid_state:
                if (_last_velocity_dict['elapsed_seconds'] - self._time_last_valid_state) > self._below_threshold_max_time:
                    # Game over. The actor has been "blocked" for too long
                    self.failure_count += 1

                    # record event
                    _last_location_dict = CriteriaManager.get_location(
                        self.actor)
                    vehicle_location = _last_location_dict['value']
                    blocked_event = ViolationEvent(
                        event_type=ViolationEventType.VEHICLE_BLOCKED, location=vehicle_location)
                    ActorSpeedAboveThresholdTest._set_event_message(
                        blocked_event, vehicle_location)
                    self.list_traffic_events.append(blocked_event)
            else:
                self._time_last_valid_state = _last_velocity_dict['elapsed_seconds']

    @staticmethod
    def _set_event_message(event, location):
        """
        Sets the message of the event
        """

        event.set_message('Agent got blocked at (x={}, y={}, z={})'.format(round(location.x, 3),
                                                                           round(
                                                                               location.y, 3),
                                                                           round(location.z, 3)))


class KeepLaneTest(Criterion):
    """
    This class contains an atomic test for keeping lane.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """

    def __init__(self, actor, optional=False, name="CheckKeepLane"):
        """
        Construction with sensor setup
        """
        super(KeepLaneTest, self).__init__(name, actor, 0, None, optional)

        world = self.actor.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self._lane_sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=self.actor)
        self._lane_sensor.listen(
            lambda event: self._count_lane_invasion(weakref.ref(self), event))
        CriteriaManager._carla_adapter.actor_list.append(self._lane_sensor)

    def update(self):
        """
        Check lane invasion count
        """
        super(KeepLaneTest, self).update()

    @staticmethod
    def _count_lane_invasion(weak_self, event):
        """
        Callback to update lane invasion count
        """
        self = weak_self()
        if not self:
            return
        self.failure_count += 1
        event_type = ViolationEventType.LANE_INVASION
        location = self.actor.get_location()
        lane_invasion_event = ViolationEvent(
            event_type=event_type, location=location)
        self.list_traffic_events.append(lane_invasion_event)


class OffRoadTest(Criterion):

    """
    Atomic containing a test to detect when an actor deviates from the driving lanes. This atomic can
    fail when actor has spent a specific time outside driving lanes (defined by OpenDRIVE). Simplified
    version of OnSidewalkTest, and doesn't relly on waypoints with *Sidewalk* lane types

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        duration (float): Time spent at sidewalks before the atomic fails.
            If terminate_on_failure isn't active, this is ignored.
        optional (bool): If True, the result is not considered for an overall pass/fail result
            when using the output argument
        terminate_on_failure (bool): If True, the atomic will fail when the duration condition has been met.
    """

    def __init__(self, actor, duration=0, optional=False, name="OffRoadTest"):
        """
        Setup of the variables
        """
        super(OffRoadTest, self).__init__(name, actor, 0, None, optional)

        self._offroad = False

        self._duration = duration
        self._prev_time = None
        self._time_offroad = 0

    def update(self):
        """
        First, transforms the actor's current position to its corresponding waypoint. This is
        filtered to only use waypoints of type Driving or Parking. Depending on these results,
        the actor will be considered to be outside (or inside) driving lanes.

        returns:
            py_trees.common.Status.FAILURE: when the actor has spent a given duration outside driving lanes
            py_trees.common.Status.RUNNING: the rest of the time
        """
        super(OffRoadTest, self).update()

        _last_location_dict = CriteriaManager.get_location(self.actor)
        current_location = _last_location_dict['value']

        # Get the waypoint at the current location to see if the actor is offroad
        drive_waypoint = CriteriaManager._carla_adapter.world.get_map().get_waypoint(
            current_location,
            project_to_road=False
        )
        park_waypoint = CriteriaManager._carla_adapter.world.get_map().get_waypoint(
            current_location,
            project_to_road=False,
            lane_type=carla.LaneType.Parking
        )
        if drive_waypoint or park_waypoint:
            self._offroad = False
        else:
            self._offroad = True

        # Counts the time offroad
        if self._offroad:
            if self._prev_time is None:
                self._prev_time = _last_location_dict['elapsed_seconds']
            else:
                curr_time = _last_location_dict['elapsed_seconds']
                self._time_offroad += curr_time - self._prev_time
                self._prev_time = curr_time
        else:
            self._prev_time = None

        if self._time_offroad > self._duration:
            self.failure_count += 1
            event_type = ViolationEventType.OFF_ROAD
            location = self.actor.get_location()
            off_road_event = ViolationEvent(
                event_type=event_type, location=location)
            self.list_traffic_events.append(off_road_event)


class OnSidewalkTest(Criterion):

    """
    Atomic containing a test to detect sidewalk invasions of a specific actor. This atomic can
    fail when actor has spent a specific time outside driving lanes (defined by OpenDRIVE).

    Args:
        actor (carla.Actor): CARLA actor to be used for this test
        duration (float): Time spent at sidewalks before the atomic fails.
            If terminate_on_failure isn't active, this is ignored.
        optional (bool): If True, the result is not considered for an overall pass/fail result
            when using the output argument
        terminate_on_failure (bool): If True, the atomic will fail when the duration condition has been met.
    """

    def __init__(self, actor, duration=0, optional=False, name="OnSidewalkTest"):
        """
        Construction with sensor setup
        """
        super(OnSidewalkTest, self).__init__(name, actor, 0, None, optional)

        self._actor = actor
        self._map = CriteriaManager._carla_adapter.world.get_map()
        self._onsidewalk_active = False
        self._outside_lane_active = False

        self._wrong_sidewalk_distance = 0
        self._wrong_outside_lane_distance = 0
        self._sidewalk_start_location = None
        self._outside_lane_start_location = None
        self._duration = duration
        self._prev_time = None
        self._time_outside_lanes = 0

    def update(self):
        """
        First, transforms the actor's current position as well as its four corners to their
        corresponding waypoints. Depending on their lane type, the actor will be considered to be
        outside (or inside) driving lanes.

        returns:
            py_trees.common.Status.FAILURE: when the actor has spent a given duration outside
                driving lanes and terminate_on_failure is active
            py_trees.common.Status.RUNNING: the rest of the time
        """

        # Some of the vehicle parameters
        _last_transform_dict = CriteriaManager.get_transform(self._actor)
        current_tra = _last_transform_dict['value']
        current_loc = current_tra.location
        current_wp = self._map.get_waypoint(
            current_loc, lane_type=carla.LaneType.Any)

        # Case 1) Car center is at a sidewalk
        if current_wp.lane_type == carla.LaneType.Sidewalk:
            if not self._onsidewalk_active:
                self._onsidewalk_active = True
                self._sidewalk_start_location = current_loc

        # Case 2) Not inside allowed zones (Driving and Parking)
        elif current_wp.lane_type != carla.LaneType.Driving \
                and current_wp.lane_type != carla.LaneType.Parking:

            # Get the vertices of the vehicle
            heading_vec = current_tra.get_forward_vector()
            heading_vec.z = 0
            heading_vec = heading_vec / \
                math.sqrt(math.pow(heading_vec.x, 2) +
                          math.pow(heading_vec.y, 2))
            perpendicular_vec = carla.Vector3D(-heading_vec.y,
                                               heading_vec.x, 0)

            extent = self.actor.bounding_box.extent
            x_boundary_vector = heading_vec * extent.x
            y_boundary_vector = perpendicular_vec * extent.y

            bbox = [
                current_loc +
                carla.Location(x_boundary_vector - y_boundary_vector),
                current_loc +
                carla.Location(x_boundary_vector + y_boundary_vector),
                current_loc +
                carla.Location(-1 * x_boundary_vector - y_boundary_vector),
                current_loc + carla.Location(-1 * x_boundary_vector + y_boundary_vector)]

            bbox_wp = [
                self._map.get_waypoint(bbox[0], lane_type=carla.LaneType.Any),
                self._map.get_waypoint(bbox[1], lane_type=carla.LaneType.Any),
                self._map.get_waypoint(bbox[2], lane_type=carla.LaneType.Any),
                self._map.get_waypoint(bbox[3], lane_type=carla.LaneType.Any)]

            # Case 2.1) Not quite outside yet
            if bbox_wp[0].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking) \
                or bbox_wp[1].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking) \
                or bbox_wp[2].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking) \
                    or bbox_wp[3].lane_type == (carla.LaneType.Driving or carla.LaneType.Parking):

                self._onsidewalk_active = False
                self._outside_lane_active = False

            # Case 2.2) At the mini Shoulders between Driving and Sidewalk
            elif bbox_wp[0].lane_type == carla.LaneType.Sidewalk \
                or bbox_wp[1].lane_type == carla.LaneType.Sidewalk \
                or bbox_wp[2].lane_type == carla.LaneType.Sidewalk \
                    or bbox_wp[3].lane_type == carla.LaneType.Sidewalk:

                if not self._onsidewalk_active:
                    self._onsidewalk_active = True
                    self._sidewalk_start_location = current_loc

            else:
                distance_vehicle_wp = current_loc.distance(
                    current_wp.transform.location)

                # Case 2.3) Outside lane
                if distance_vehicle_wp >= current_wp.lane_width / 2:

                    if not self._outside_lane_active:
                        self._outside_lane_active = True
                        self._outside_lane_start_location = current_loc

                # Case 2.4) Very very edge case (but still inside driving lanes)
                else:
                    self._onsidewalk_active = False
                    self._outside_lane_active = False

        # Case 3) Driving and Parking conditions
        else:
            # Check for false positives at junctions
            if current_wp.is_junction:
                distance_vehicle_wp = math.sqrt(
                    math.pow(current_wp.transform.location.x - current_loc.x, 2) +
                    math.pow(current_wp.transform.location.y - current_loc.y, 2))

                if distance_vehicle_wp <= current_wp.lane_width / 2:
                    self._onsidewalk_active = False
                    self._outside_lane_active = False
                # Else, do nothing, the waypoint is too far to consider it a correct position
            else:

                self._onsidewalk_active = False
                self._outside_lane_active = False

        # Counts the time offroad
        if self._onsidewalk_active or self._outside_lane_active:
            if self._prev_time is None:
                self._prev_time = _last_transform_dict['elapsed_seconds']
            else:
                curr_time = _last_transform_dict['elapsed_seconds']
                self._time_outside_lanes += curr_time - self._prev_time
                self._prev_time = curr_time
        else:
            self._prev_time = None

        # 记测试不通过一次
        if self._time_outside_lanes > self._duration:
            self.failure_count += 1

            # Register the sidewalk event
            if not self._onsidewalk_active:

                onsidewalk_event = ViolationEvent(
                    event_type=ViolationEventType.ON_SIDEWALK_INFRACTION, location=self._sidewalk_start_location)
                self._set_event_message(
                    onsidewalk_event, self._sidewalk_start_location)

                self._onsidewalk_active = False
                self.list_traffic_events.append(onsidewalk_event)

            # Register the outside of a lane event
            if not self._outside_lane_active:

                outsidelane_event = ViolationEvent(
                    event_type=ViolationEventType.OUTSIDE_LANE_INFRACTION, location=self._outside_lane_start_location)
                self._set_event_message(
                    outsidelane_event, self._outside_lane_start_location)

                self._outside_lane_active = False
                self.list_traffic_events.append(outsidelane_event)

    def _set_event_message(self, event, location):
        """
        Sets the message of the event
        """
        if event.get_type() == ViolationEventType.ON_SIDEWALK_INFRACTION:
            message_start = 'Agent invaded the sidewalk'
        else:
            message_start = 'Agent went outside the lane'

        event.set_message(
            '{} starting at (x={}, y={}, z={})'.format(
                message_start,
                round(location.x, 3),
                round(location.y, 3),
                round(location.z, 3)))


class WrongLaneTest(Criterion):

    """
    This class contains an atomic test to detect invasions to wrong direction lanes.

    Important parameters:
    - actor: CARLA actor to be used for this test
    - optional [optional]: If True, the result is not considered for an overall pass/fail result
    """
    MAX_ALLOWED_ANGLE = 120.0
    MAX_ALLOWED_WAYPOINT_ANGLE = 150.0

    def __init__(self, actor, optional=False, name="WrongLaneTest"):
        """
        Construction with sensor setup
        """
        super(WrongLaneTest, self).__init__(name, actor, 0, None, optional)

        self._actor = actor
        self._map = CriteriaManager._carla_adapter.world.get_map()
        self._last_lane_id = None
        self._last_road_id = None

        self._in_lane = True
        self._wrong_distance = 0
        self._actor_location = self._actor.get_location()
        self._previous_lane_waypoint = self._map.get_waypoint(
            self._actor.get_location())
        self._wrong_lane_start_location = None

    def update(self):
        """
        Check lane invasion count
        """

        lane_waypoint = self._map.get_waypoint(self._actor.get_location())
        current_lane_id = lane_waypoint.lane_id
        current_road_id = lane_waypoint.road_id

        if (self._last_road_id != current_road_id or self._last_lane_id != current_lane_id) \
                and not lane_waypoint.is_junction:
            next_waypoint = lane_waypoint.next(2.0)[0]

            if not next_waypoint:
                return

            # The waypoint route direction can be considered continuous.
            # Therefore just check for a big gap in waypoint directions.
            previous_lane_direction = self._previous_lane_waypoint.transform.get_forward_vector()
            current_lane_direction = lane_waypoint.transform.get_forward_vector()

            p_lane_vector = np.array(
                [previous_lane_direction.x, previous_lane_direction.y])
            c_lane_vector = np.array(
                [current_lane_direction.x, current_lane_direction.y])

            waypoint_angle = math.degrees(
                math.acos(np.clip(np.dot(p_lane_vector, c_lane_vector) /
                                  (np.linalg.norm(p_lane_vector) * np.linalg.norm(c_lane_vector)), -1.0, 1.0)))

            if waypoint_angle > self.MAX_ALLOWED_WAYPOINT_ANGLE and self._in_lane:

                self.failure_count += 1
                self._in_lane = False
                self._wrong_lane_start_location = self._actor_location

            else:
                # Reset variables
                self._in_lane = True

            # Continuity is broken after a junction so check vehicle-lane angle instead
            if self._previous_lane_waypoint.is_junction:

                vector_wp = np.array([next_waypoint.transform.location.x - lane_waypoint.transform.location.x,
                                      next_waypoint.transform.location.y - lane_waypoint.transform.location.y])

                vector_actor = np.array([math.cos(math.radians(self._actor.get_transform().rotation.yaw)),
                                         math.sin(math.radians(self._actor.get_transform().rotation.yaw))])

                vehicle_lane_angle = math.degrees(
                    math.acos(np.clip(np.dot(vector_actor, vector_wp) / (np.linalg.norm(vector_wp)), -1.0, 1.0)))

                if vehicle_lane_angle > self.MAX_ALLOWED_ANGLE:

                    self.failure_count += 1
                    self._in_lane = False
                    self._wrong_lane_start_location = self._actor.get_location()

        # Keep adding "meters" to the counter
        distance_vector = self._actor.get_location() - self._actor_location
        distance = math.sqrt(math.pow(distance_vector.x, 2) +
                             math.pow(distance_vector.y, 2))

        if distance >= 0.02:  # Used to avoid micro-changes adding add to considerable sums
            _last_location_dict = CriteriaManager.get_location(self._actor)
            self._actor_location = _last_location_dict['value']

            if not self._in_lane and not lane_waypoint.is_junction:
                self._wrong_distance += distance

        # Register the event
        if self._in_lane and self._wrong_distance > 0:

            wrong_way_event = ViolationEvent(
                event_type=ViolationEventType.WRONG_WAY_INFRACTION, location=self._wrong_lane_start_location)
            self._set_event_message(wrong_way_event, self._wrong_lane_start_location,
                                    self._wrong_distance, current_road_id, current_lane_id)

            self.list_traffic_events.append(wrong_way_event)
            self._wrong_distance = 0

        # Remember the last state
        self._last_lane_id = current_lane_id
        self._last_road_id = current_road_id
        self._previous_lane_waypoint = lane_waypoint

    def _set_event_message(self, event, location, distance, road_id, lane_id):
        """
        Sets the message of the event
        """

        event.set_message(
            "Agent invaded a lane in opposite direction for {} meters, starting at (x={}, y={}, z={}). "
            "road_id={}, lane_id={}".format(
                round(distance, 3),
                round(location.x, 3),
                round(location.y, 3),
                round(location.z, 3),
                road_id,
                lane_id))


class InRadiusRegionTest(Criterion):
    # TODO:检查某个actor是否顺利到达了指定的终点
    pass


class RunningRedLightTest(Criterion):

    """
    Check if an actor is running a red light

    Important parameters:
    - actor: CARLA actor to be used for this test
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    DISTANCE_LIGHT = 15  # m

    def __init__(self, actor, name="RunningRedLightTest"):
        """
        Init
        """
        super(RunningRedLightTest, self).__init__(name, actor, 0)
        self._actor = actor
        self._world = actor.get_world()
        self._map = CriteriaManager._carla_adapter.world.get_map()
        self._list_traffic_lights = []
        self._last_red_light_id = None

        all_actors = self._world.get_actors()
        for _actor in all_actors:
            if 'traffic_light' in _actor.type_id:
                center, waypoints = self.get_traffic_light_waypoints(_actor)
                self._list_traffic_lights.append((_actor, center, waypoints))

    # pylint: disable=no-self-use
    def is_vehicle_crossing_line(self, seg1, seg2):
        """
        check if vehicle crosses a line segment
        """
        line1 = shapely.geometry.LineString(
            [(seg1[0].x, seg1[0].y), (seg1[1].x, seg1[1].y)])
        line2 = shapely.geometry.LineString(
            [(seg2[0].x, seg2[0].y), (seg2[1].x, seg2[1].y)])
        inter = line1.intersection(line2)

        return not inter.is_empty

    def update(self):
        """
        Check if the actor is running a red light
        """

        _last_transform_dict = CriteriaManager.get_transform(self._actor)
        transform = _last_transform_dict['value']
        location = transform.location
        if location is None:
            return

        veh_extent = self._actor.bounding_box.extent.x

        tail_close_pt = self.rotate_point(
            carla.Vector3D(-0.8 * veh_extent, 0.0, location.z), transform.rotation.yaw)
        tail_close_pt = location + carla.Location(tail_close_pt)

        tail_far_pt = self.rotate_point(
            carla.Vector3D(-veh_extent - 1, 0.0, location.z), transform.rotation.yaw)
        tail_far_pt = location + carla.Location(tail_far_pt)

        for traffic_light, center, waypoints in self._list_traffic_lights:

            center_loc = carla.Location(center)

            if self._last_red_light_id and self._last_red_light_id == traffic_light.id:
                continue
            if center_loc.distance(location) > self.DISTANCE_LIGHT:
                continue
            if traffic_light.state != carla.TrafficLightState.Red:
                continue

            for wp in waypoints:

                tail_wp = self._map.get_waypoint(tail_far_pt)

                # Calculate the dot product (Might be unscaled, as only its sign is important)
                _last_transform_dict = CriteriaManager.get_transform(
                    self._actor)
                ve_dir = _last_transform_dict['value'].get_forward_vector()
                wp_dir = wp.transform.get_forward_vector()
                dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

                # Check the lane until all the "tail" has passed
                if tail_wp.road_id == wp.road_id and tail_wp.lane_id == wp.lane_id and dot_ve_wp > 0:
                    # This light is red and is affecting our lane
                    yaw_wp = wp.transform.rotation.yaw
                    lane_width = wp.lane_width
                    location_wp = wp.transform.location

                    lft_lane_wp = self.rotate_point(carla.Vector3D(
                        0.4 * lane_width, 0.0, location_wp.z), yaw_wp + 90)
                    lft_lane_wp = location_wp + carla.Location(lft_lane_wp)
                    rgt_lane_wp = self.rotate_point(carla.Vector3D(
                        0.4 * lane_width, 0.0, location_wp.z), yaw_wp - 90)
                    rgt_lane_wp = location_wp + carla.Location(rgt_lane_wp)

                    # Is the vehicle traversing the stop line?
                    if self.is_vehicle_crossing_line((tail_close_pt, tail_far_pt), (lft_lane_wp, rgt_lane_wp)):

                        self.failure_count += 1
                        location = traffic_light.get_transform().location
                        red_light_event = ViolationEvent(
                            event_type=ViolationEventType.TRAFFIC_LIGHT_INFRACTION, location=location)
                        red_light_event.set_message(
                            "Agent ran a red light {} at (x={}, y={}, z={})".format(
                                traffic_light.id,
                                round(location.x, 3),
                                round(location.y, 3),
                                round(location.z, 3)))

                        self.list_traffic_events.append(red_light_event)
                        self._last_red_light_id = traffic_light.id
                        break

    def rotate_point(self, point, angle):
        """
        rotate a given point by a given angle
        """
        x_ = math.cos(math.radians(angle)) * point.x - \
            math.sin(math.radians(angle)) * point.y
        y_ = math.sin(math.radians(angle)) * point.x + \
            math.cos(math.radians(angle)) * point.y
        return carla.Vector3D(x_, y_, point.z)

    def get_traffic_light_waypoints(self, traffic_light):
        """
        get area of a given traffic light
        """
        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(
            traffic_light.trigger_volume.location)

        # Discretize the trigger box into points
        area_ext = traffic_light.trigger_volume.extent
        # 0.9 to avoid crossing to adjacent lanes
        x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)

        area = []
        for x in x_values:
            point = self.rotate_point(
                carla.Vector3D(x, 0, area_ext.z), base_rot)
            point_location = area_loc + carla.Location(x=point.x, y=point.y)
            area.append(point_location)

        # Get the waypoints of these points, removing duplicates
        ini_wps = []
        for pt in area:
            wpx = self._map.get_waypoint(pt)
            # As x_values are arranged in order, only the last one has to be checked
            if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
                ini_wps.append(wpx)

        # Advance them until the intersection
        wps = []
        for wpx in ini_wps:
            while not wpx.is_intersection:
                next_wp = wpx.next(0.5)[0]
                if next_wp and not next_wp.is_intersection:
                    wpx = next_wp
                else:
                    break
            wps.append(wpx)

        return area_loc, wps


class RunningStopTest(Criterion):

    """
    Check if an actor is running a stop sign

    Important parameters:
    - actor: CARLA actor to be used for this test
    - terminate_on_failure [optional]: If True, the complete scenario will terminate upon failure of this test
    """
    PROXIMITY_THRESHOLD = 50.0  # meters
    SPEED_THRESHOLD = 0.1
    WAYPOINT_STEP = 1.0  # meters

    def __init__(self, actor, name="RunningStopTest"):
        """
        """
        super(RunningStopTest, self).__init__(name, actor, 0)
        self._actor = actor
        self._world = actor.get_world()
        self._map = CriteriaManager._carla_adapter.world.get_map()
        self._list_stop_signs = []
        self._target_stop_sign = None
        self._stop_completed = False
        self._affected_by_stop = False

        all_actors = self._world.get_actors()
        for _actor in all_actors:
            if 'traffic.stop' in _actor.type_id:
                self._list_stop_signs.append(_actor)

    @staticmethod
    def point_inside_boundingbox(point, bb_center, bb_extent):
        """
        X
        :param point:
        :param bb_center:
        :param bb_extent:
        :return:
        """

        # pylint: disable=invalid-name
        A = carla.Vector2D(bb_center.x - bb_extent.x,
                           bb_center.y - bb_extent.y)
        B = carla.Vector2D(bb_center.x + bb_extent.x,
                           bb_center.y - bb_extent.y)
        D = carla.Vector2D(bb_center.x - bb_extent.x,
                           bb_center.y + bb_extent.y)
        M = carla.Vector2D(point.x, point.y)

        AB = B - A
        AD = D - A
        AM = M - A
        am_ab = AM.x * AB.x + AM.y * AB.y
        ab_ab = AB.x * AB.x + AB.y * AB.y
        am_ad = AM.x * AD.x + AM.y * AD.y
        ad_ad = AD.x * AD.x + AD.y * AD.y

        return am_ab > 0 and am_ab < ab_ab and am_ad > 0 and am_ad < ad_ad

    def is_actor_affected_by_stop(self, actor, stop, multi_step=20):
        """
        Check if the given actor is affected by the stop
        """
        affected = False
        # first we run a fast coarse test
        current_location = actor.get_location()
        stop_location = stop.get_transform().location
        if stop_location.distance(current_location) > self.PROXIMITY_THRESHOLD:
            return affected

        stop_t = stop.get_transform()
        transformed_tv = stop_t.transform(stop.trigger_volume.location)

        # slower and accurate test based on waypoint's horizon and geometric test
        list_locations = [current_location]
        waypoint = self._map.get_waypoint(current_location)
        for _ in range(multi_step):
            if waypoint:
                next_wps = waypoint.next(self.WAYPOINT_STEP)
                if not next_wps:
                    break
                waypoint = next_wps[0]
                if not waypoint:
                    break
                list_locations.append(waypoint.transform.location)

        for actor_location in list_locations:
            if self.point_inside_boundingbox(actor_location, transformed_tv, stop.trigger_volume.extent):
                affected = True

        return affected

    def _scan_for_stop_sign(self):
        target_stop_sign = None

        _last_transform_dict = CriteriaManager.get_transform(self._actor)
        ve_tra = _last_transform_dict['value']
        ve_dir = ve_tra.get_forward_vector()

        wp = self._map.get_waypoint(ve_tra.location)
        wp_dir = wp.transform.get_forward_vector()

        dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

        if dot_ve_wp > 0:  # Ignore all when going in a wrong lane
            for stop_sign in self._list_stop_signs:
                if self.is_actor_affected_by_stop(self._actor, stop_sign):
                    # this stop sign is affecting the vehicle
                    target_stop_sign = stop_sign
                    break

        return target_stop_sign

    def update(self):
        """
        Check if the actor is running a red light
        """

        location = self._actor.get_location()
        if location is None:
            return

        if not self._target_stop_sign:
            # scan for stop signs
            self._target_stop_sign = self._scan_for_stop_sign()
        else:
            # we were in the middle of dealing with a stop sign
            if not self._stop_completed:
                # did the ego-vehicle stop?
                _last_velocity_dict = CriteriaManager.get_velocity(self._actor)
                current_speed = _last_velocity_dict['value']
                if current_speed < self.SPEED_THRESHOLD:
                    self._stop_completed = True

            if not self._affected_by_stop:
                stop_location = self._target_stop_sign.get_location()
                stop_extent = self._target_stop_sign.trigger_volume.extent

                if self.point_inside_boundingbox(location, stop_location, stop_extent):
                    self._affected_by_stop = True

            if not self.is_actor_affected_by_stop(self._actor, self._target_stop_sign):
                # is the vehicle out of the influence of this stop sign now?
                if not self._stop_completed and self._affected_by_stop:
                    # did we stop?
                    self.failure_count += 1
                    stop_location = self._target_stop_sign.get_transform().location
                    running_stop_event = ViolationEvent(
                        event_type=ViolationEventType.STOP_INFRACTION, location=stop_location)
                    running_stop_event.set_message(
                        "Agent ran a stop with id={} at (x={}, y={}, z={})".format(
                            self._target_stop_sign.id,
                            round(stop_location.x, 3),
                            round(stop_location.y, 3),
                            round(stop_location.z, 3)))

                    self.list_traffic_events.append(running_stop_event)

                # reset state
                self._target_stop_sign = None
                self._stop_completed = False
                self._affected_by_stop = False
