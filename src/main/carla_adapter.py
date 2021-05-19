import threading
import random
import carla
import math
from src.tools.auto_criteria import *
from src.tools.utils import RepeatedTimer
import time
import src.tools.global_var as glv


class CarlaAdapter:
    def __init__(self, ip_address):
        self.client = carla.Client(ip_address, 2000)
        self.client.set_timeout(10.0)  # 防止性能太差，无法建立连接
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.spectator = None
        self.actor_list = []

    def set_map(self, map):
        try:
            if self.world.get_map().name == map:
                print("Map has already been loaded")
            else:
                self.client.load_world(map)
                self.world = self.client.get_world()
                print("New map loaded")
        except Exception as exception:
            print("Load {} failed:{}".format(map, exception))

    def init(self):
        pass

    def run(self):
        pass

    # Get Spectator and spawn corresponding Sensor

    def set_spectator(self):
        self.spectator = self.world.get_spectator()
        sensor_blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        sensor_blueprint.set_attribute('image_size_x', '960')
        sensor_blueprint.set_attribute('image_size_y', '540')
        sensor_blueprint.set_attribute('fov', '110')
        # Set the time in seconds between sensor captures
        sensor_blueprint.set_attribute('sensor_tick', '0.0')
        transform = carla.Transform(
            carla.Location(x=-4, z=1.9))
        sensor = self.world.spawn_actor(
            sensor_blueprint, transform, attach_to=self.spectator)
        self.actor_list.append(sensor)
        sensor.listen(lambda data: glv.get("queue_global").put(data))

        print(
            "Spectator created:{}-{}".format(sensor.id, sensor.type_id))

    # WASD Move spectator

    def set_spectator_transform(self, ins):
        print(ins)
        transform = None
        if ins == 'key_w':
            transform = self.get_transform_offset(x=0.5)
        elif ins == 'key_s':
            transform = self.get_transform_offset(x=-0.5)
        elif ins == 'key_d':
            transform = self.get_transform_offset(y=0.5)
        elif ins == 'key_a':
            transform = self.get_transform_offset(y=-0.5)
        elif ins == 'key_q':
            transform = self.get_transform_offset(z=-0.5)
        elif ins == 'key_e':
            transform = self.get_transform_offset(z=0.5)
        elif ins == 'drag_r':
            transform = self.get_transform_drag(angleX=5)
        elif ins == 'drag_l':
            transform = self.get_transform_drag(angleX=-5)
        elif ins == 'drag_u':
            transform = self.get_transform_drag(angleY=5)
        elif ins == 'drag_d':
            transform = self.get_transform_drag(angleY=-5)
        else:
            print("key not supported")
            return
        self.spectator.set_transform(transform)
        print(transform)
        print("Sensor location: --")

    # 处理WASD键的前后左右移动

    def get_transform_offset(self, x=0, y=0, z=0):
        # 角度转弧度
        pitch_radius = math.radians(
            self.spectator.get_transform().rotation.pitch)  # y
        # z
        yaw_radius = math.radians(-self.spectator.get_transform().rotation.yaw)
        # 绕y轴旋转
        x_1 = x * math.cos(pitch_radius) - z * math.sin(pitch_radius)
        z_1 = x * math.sin(pitch_radius) + z * math.cos(pitch_radius)
        y_1 = y
        # 绕z轴旋转
        y_2 = y_1 * math.cos(yaw_radius) - x_1 * math.sin(yaw_radius)
        x_2 = y_1 * math.sin(yaw_radius) + x_1 * math.cos(yaw_radius)
        z_2 = z_1
        # 根据偏移，设置位置
        location = carla.Location(x_2, y_2, z_2) + \
            self.spectator.get_location()
        transform = carla.Transform(
            location, self.spectator.get_transform().rotation)
        return transform

    # 处理鼠标拖拽左右拖拽（yaw）和上下拖拽（pitch）

    def get_transform_drag(self, angleX=0, angleY=0):
        origin_rotation = self.spectator.get_transform().rotation
        rotation = carla.Rotation(yaw=angleX + origin_rotation.yaw,
                                  pitch=angleY + origin_rotation.pitch, roll=origin_rotation.roll)
        transform = carla.Transform(self.spectator.get_location(), rotation)
        return transform

    def stop(self):
        if len(self.actor_list) > 0:
            actor_ids = [x.id for x in self.actor_list]
            self.client.apply_batch(
                [carla.command.DestroyActor(x) for x in actor_ids])
        self.actor_list = []

        # actor_list = [x for x in self.world.get_actors() if x.type_id.split(".")[0] == "vehicle" or x.type_id.split(
        #     ".")[0] == "walker" or x.type_id.split(".")[0] == "sensor" or x.type_id.split(".")[0] == "controller"]
        # self.client.apply_batch([carla.command.DestroyActor(x)
        #                          for x in actor_list])


class AdaptedActor:
    def __init__(self, world, name=None, blueprint=None, actor_type="Vehicle"):
        self.world = world
        self.map = self.world.get_map()
        self.carla_actor = None
        self.name = name
        self.blueprint = blueprint
        self.actor_type = actor_type
        self.start_transform = None
        self.target_transform = None
        self._middle_transforms = []

    def set_start_position(self, position, position_type="COORDINATE"):
        self.start_transform = self.__get_valid_transform(
            position, position_type)
        self.start_transform.location.z += 0.4

    def set_target_position(self, position, position_type="COORDINATE"):
        self.target_transform = self.__get_valid_transform(
            position, position_type)

    def set_middle_positions(self, positions, position_type="COORDINATE"):
        for position in positions:
            transform = self.__get_valid_transform(
                position, position_type)
            self._middle_transforms.append(transform)

    @property
    def path_transform_list(self):
        path_transform_list = [self.start_transform] + \
            self._middle_transforms + [self.target_transform]
        return path_transform_list

    def __get_valid_transform(self, position, position_type):
        if position_type == "COORDINATE":
            location = carla.Location(position[0], position[1], 0)
            # auto choose the nearest waypoint
            transform = self.__get_waypoint_by_location(
                location).transform
        else:
            transform = self.__get_waypoint_by_mixed_laneid(
                position[0], position[1], position[2]).transform

        return transform

    def __get_waypoint_by_location(self, location):
        if self.actor_type == "Vehicle":
            waypoint = self.map.get_waypoint(
                location, lane_type=carla.LaneType.Driving)
        elif self.actor_type == "Pedestrian":
            waypoint = self.map.get_waypoint(
                location, lane_type=carla.LaneType.Sidewalk)
        else:
            print("Error:wrong actor type")
            waypoint = self.map.get_waypoint(location)
        return waypoint

    def __get_waypoint_by_mixed_laneid(self, road_id, lane_id, length):
        # road_id, lane_id = mixed_lane_id.split('.')
        waypoint = self.map.get_waypoint_xodr(
            int(road_id), int(lane_id), length)
        return waypoint

    def draw_tips(self):
        # Debug: draw tips on the Carla world
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        self.world.debug.draw_string(self.start_transform.location, "[{}]START".format(self.name), draw_shadow=False,
                                     color=carla.Color(r=r, g=g, b=b), life_time=100,
                                     persistent_lines=True)
        self.world.debug.draw_string(self.target_transform.location, "[{}]TARGET".format(self.name), draw_shadow=False,
                                     color=carla.Color(r=r, g=g, b=b), life_time=100,
                                     persistent_lines=True)

    def info(self):
        def __get_float_value(x_str):
            x_float = float(x_str)
            return round(x_float, 2)
        info_string = "[{}]:({},{},{})->({},{},{})".format(self.name, __get_float_value(self.start_transform.location.x),
                                                           __get_float_value(
                                                               self.start_transform.location.y),
                                                           __get_float_value(
                                                               self.start_transform.location.z),
                                                           __get_float_value(
                                                               self.target_transform.location.x),
                                                           __get_float_value(
                                                               self.target_transform.location.y),
                                                           __get_float_value(self.target_transform.location.z))
        return info_string

    def spawn(self):
        try:
            actor = self.world.spawn_actor(
                self.blueprint, self.start_transform)
            self.carla_actor = actor
            return actor
        except Exception as exception:
            print("Spawn error:{}".format(exception))
            return None


class AdaptedVehicle(AdaptedActor):
    def __init__(self, world, name=None, blueprint=None, speed=None):
        AdaptedActor.__init__(self, world, name, blueprint, "Vehicle")
        self.car = []
        self.bus = []
        self.van = []
        self.truck = []
        self.bicycle = []
        self.motorbicycle = []
        self.speed = speed
        self.info_collector_thread = None
        self.infomation_dict = {
            "throttle": [],
            "steer": [],
            "location": []
        }

    def set_speed(self):
        if self.speed is not None:
            speed = float(self.speed)
            if speed >= 0:
                self.carla_actor.enable_constant_velocity(
                    carla.Vector3D(speed, 0, 0))
                print("set {} speed to {}".format(
                    self.name, speed))

    def use_auto(self):
        if self.speed is not None:
            speed = float(self.speed)
            if speed >= 0:
                return False
            else:
                return True
        else:
            return False

    def stop(self):
        if self.carla_actor is not None:
            self.carla_actor.enable_constant_velocity(carla.Vector3D(0, 0, 0))

    def set_random_target(self):
        spawn_points = self.map.get_spawn_points()
        random.shuffle(spawn_points)
        for transform in spawn_points:
            if self.start_transform is not None:
                if abs(self.start_transform.location.x - transform.location.x) < 50 or abs(self.start_transform.location.y - transform.location.y) < 50:
                    continue
            self.target_transform = transform
            break

    def start_to_collect(self):
        # collector method
        def collect_infomation(actor, info_dict):
            # collect actor infomation
            control = actor.get_control()
            location = actor.get_location()
            info_dict["throttle"].append(control.throttle)
            info_dict["steer"].append(control.steer)
            info_dict["location"].append(location)
        # start to collect actor infomation
        self.info_collector_thread = RepeatedTimer(
            1, collect_infomation, self.carla_actor, self.infomation_dict)  # auto starts

    def stop_collect(self):
        # stop to collect actor infomation
        if self.info_collector_thread:
            self.info_collector_thread.stop()

    def create_criterias(self):
        # 注册各种标准
        _ego_max_velocity_allowed = 20       # Maximum allowed velocity [km/h]
        max_velocity_test = MaxVelocityTest(
            self.carla_actor, _ego_max_velocity_allowed)
        _avg_velocity_success = 10
        average_velocity_test = AverageVelocityTest(
            self.carla_actor, _avg_velocity_success)
        collision_test = CollisionTest(actor=self.carla_actor)
        _LOWEST_SPEED_THRESHOLD = 1         # [m/s]
        _BELOW_THRESHOLD_MAX_TIME = 3
        agent_block_test = ActorSpeedAboveThresholdTest(
            self.carla_actor, _LOWEST_SPEED_THRESHOLD, _BELOW_THRESHOLD_MAX_TIME)
        keep_lane_test = KeepLaneTest(self.carla_actor)
        off_road_test = OffRoadTest(self.carla_actor)
        on_sidewalk_test = OnSidewalkTest(self.carla_actor)
        wrong_lane_test = WrongLaneTest(self.carla_actor)
        running_red_light_test = RunningRedLightTest(self.carla_actor)
        running_stop_test = RunningStopTest(self.carla_actor)

        return [max_velocity_test, average_velocity_test, collision_test, agent_block_test, keep_lane_test,
                off_road_test, on_sidewalk_test, wrong_lane_test, running_red_light_test, running_stop_test]

    def start_collision_detect(self, state_callback):
        def attach_collision_sensor(collision_sensor, state_callback):
            HAS_STOPPED = False

            def callback(event):
                nonlocal HAS_STOPPED
                if HAS_STOPPED == False:
                    print("***Collision detected:{}***".format(event))
                    # stop the test
                    state_callback("STOP")
                HAS_STOPPED = True

            collision_sensor.listen(lambda event: callback(event))
            # TODO: to keep sensor alive
            while True:
                time.sleep(10)

        blueprint = self.world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = self.world.spawn_actor(
            blueprint, carla.Transform(), attach_to=self.carla_actor)

        thread = threading.Thread(target=attach_collision_sensor, args=(
            collision_sensor, state_callback))
        thread.start()

        return collision_sensor


class AdaptedPedestrian(AdaptedActor):
    def __init__(self, world, name, blueprint):
        AdaptedActor.__init__(self, world, name, blueprint, "Pedestrian")

    def start_ai_walk(self):
        if self.carla_actor is not None:
            walker_controller_bp = self.world.get_blueprint_library().find(
                'controller.ai.walker')
            try:
                ai_wakler = self.world.spawn_actor(
                    walker_controller_bp, self.start_transform, self.carla_actor)
                ai_wakler.start()
                target_location = self.target_transform.location
                ai_wakler.go_to_location(target_location)
                ai_wakler.set_max_speed(2)
                return ai_wakler
            except Exception as exception:
                print("Spawn AI Walker error:{}".format(exception))
                return None
        else:
            print("Error: actor not spawned yet")
            return None


class AdaptedObstacle(AdaptedActor):
    def __init__(self, world, blueprint):
        AdaptedActor.__init__(self, world, None, blueprint, "Obstacle")
