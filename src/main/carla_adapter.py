import threading
import random
import carla
from src.scenest_parser.ast.base.weathers import WeatherContinuousIndex
from src.tools import utils
import math
from src.tools.agents.navigation.behavior_agent import BehaviorAgent

# TODO: (optimize)reduce the usage of world.get_map()


class CarlaAdapter:
    def __init__(self, ip_address):
        self.client = carla.Client(ip_address, 2000)
        self.world = None
        self.actor_list = []
        self.autopilot_batch = []
        self.vehicle_agent_dict = {}
        self.npc_thread = None
        self.blueprint_library = None
        self.id_name_map = {}
        self.spectator = None
        self.traffic_manager = self.client.get_trafficmanager(9988)

    # Map

    def set_map(self, map):
        try:
            self.client.set_timeout(10.0)  # 防止性能太差，无法建立连接
            self.world = self.client.get_world()
            if self.world.get_map().name == map:
                print("Map has already been loaded")
            else:
                self.client.load_world(map)
                self.world = self.client.get_world()
                print("New map loaded")
        except Exception as exception:
            print("Load {} failed:{}".format(map, exception))
        self.blueprint_library = self.world.get_blueprint_library()

    # Get Spectator and spawn corresponding Sensor

    def set_spectator(self):
        self.spectator = self.world.get_spectator()
        sensor_blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Modify the attributes of the blueprint to set image resolution and field of view.
        sensor_blueprint.set_attribute('image_size_x', '960')
        sensor_blueprint.set_attribute('image_size_y', '540')
        sensor_blueprint.set_attribute('fov', '110')
        # Set the time in seconds between sensor captures
        sensor_blueprint.set_attribute('sensor_tick', '0.05')
        transform = carla.Transform(
            carla.Location(x=-4, z=1.9))
        sensor = self.world.spawn_actor(
            sensor_blueprint, transform, attach_to=self.spectator)
        self.actor_list.append(sensor)

        print(
            "Sensor created:{}-{}".format(sensor.id, sensor.type_id))

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
            transform = self.get_transform_drag(angleX=-1)
        elif ins == 'drag_l':
            transform = self.get_transform_drag(angleX=1)
        elif ins == 'drag_u':
            transform = self.get_transform_drag(angleY=1)
        elif ins == 'drag_d':
            transform = self.get_transform_drag(angleY=-1)
        else:
            print("key not supported")
            return
        self.spectator.set_transform(transform)
        print(transform)
        print("Sensor location: --")
    
    # 处理WASD键的前后左右移动

    def get_transform_offset(self, x=0, y=0, z=0):
        #角度转弧度
        pitch_radius = math.radians(self.spectator.get_transform().rotation.pitch) #y
        yaw_radius = math.radians(-self.spectator.get_transform().rotation.yaw)  #z
        #绕y轴旋转
        x_1 = x * math.cos(pitch_radius) - z * math.sin(pitch_radius)
        z_1 = x * math.sin(pitch_radius) + z * math.cos(pitch_radius)
        y_1 = y
        #绕z轴旋转
        y_2 = y_1 * math.cos(yaw_radius) - x_1 * math.sin(yaw_radius)
        x_2 = y_1 * math.sin(yaw_radius) + x_1 * math.cos(yaw_radius)
        z_2 = z_1          
        #根据偏移，设置位置
        location = carla.Location(x_2, y_2, z_2) + self.spectator.get_location()
        transform = carla.Transform(location, self.spectator.get_transform().rotation)
        return transform
    
    # 处理鼠标拖拽左右拖拽（yaw）和上下拖拽（pitch）

    def get_transform_drag(self, angleX=0, angleY=0):
        origin_rotation = self.spectator.get_transform().rotation
        rotation = carla.Rotation(yaw=angleX + origin_rotation.yaw, pitch=angleY + origin_rotation.pitch, roll=origin_rotation.roll)
        transform = carla.Transform(self.spectator.get_location(), rotation)
        return transform

    # NPCVehicles

    def create_npc_vehicles(self, npcs):
        print("Number of NPCs:"+str(npcs.get_size()))
        if npcs.get_size() < 1:
            return
        spawn_batch = []
        adapted_vehicles = []

        npc_vehicles = npcs._vehicles
        for npc in npc_vehicles:
            adapted_vehicle = AdaptedVehicle(self.world, npc)
            adapted_vehicles.append(adapted_vehicle)
            spawn_batch.append(carla.command.SpawnActor(
                adapted_vehicle.blueprint, adapted_vehicle.start_transform))

        # spawn npcs together
        for index, response in enumerate(self.client.apply_batch_sync(spawn_batch, True)):
            if response.error:
                print(response.error)
                adapted_vehicles[index].carla_actor = None
            else:
                actor = self.world.get_actor(response.actor_id)
                adapted_vehicles[index].carla_actor = actor
                self.actor_list.append(actor)

        for adapted_vehicle in adapted_vehicles:
            if adapted_vehicle.carla_actor is None:
                continue
            if adapted_vehicle.use_auto():
                # set auto pilot
                self.autopilot_batch.append(carla.command.SetAutopilot(
                    adapted_vehicle.carla_actor, True, self.traffic_manager.get_port()))
            else:
                # init agent
                agent = BehaviorAgent(
                    adapted_vehicle.carla_actor, ignore_traffic_light=False, behavior='normal')
                destination_list = [
                    t.location for t in adapted_vehicle.path_transform_list]
                agent.set_many_destinations(destination_list, clean=True)
                self.vehicle_agent_dict[adapted_vehicle] = agent
            # adapted_vehicle.draw_tips() #debug
            # 维护walker_name变量名与carla中actor.id的对应关系
            self.id_name_map[str(
                adapted_vehicle.carla_actor.id)] = adapted_vehicle.name

            print(adapted_vehicle.info())

    def run_npc_vehicles(self):
        for response in self.client.apply_batch_sync(self.autopilot_batch, True):
            if response.error:
                print(response.error)
        self.npc_thread = NPCControlThread(self.vehicle_agent_dict, self.world)
        self.npc_thread.start()

    # Pedestrians

    def set_pedestrians(self, peds):
        print("Number of pedestrians:"+str(peds.get_size()))
        if peds.get_size() > 0:
            pedestrians = peds.get_pedestrians()
            for pedestrian in pedestrians:
                adapted_pedestrian = AdaptedPedestrian(self.world, pedestrian)
                adapted_pedestrian.spawn()
                # 维护walker_name变量名与carla中actor.id的对应关系
                self.id_name_map[str(
                    adapted_pedestrian.carla_actor.id)] = adapted_pedestrian.name

                self.actor_list.append(adapted_pedestrian.carla_actor)
                adapted_pedestrian.start_ai_walk()
                print(adapted_pedestrian.info())

    # Obstacles

    def set_obstacles(self, obs):
        print("Number of obstacles:"+str(obs.get_size()))
        obstacles = obs.get_obstacle()
        if obstacles.get_size() > 0:
            for obstacle in obstacles:
                adapted_obstacle = AdaptedObstacle(self.world, obstacle)
                obstacle = adapted_obstacle.spawn()
                print(adapted_obstacle.info())
                self.actor_list.append(obstacle)

    # Environment

    def set_environment(self, env):
        # get the weather of the world
        light = 0.1
        middle = 0.5
        heavy = 0.9
        weather_now = self.world.get_weather()
        if env.get_time():
            time = env.get_time().get_hour()+env.get_time().get_minute()/60
            if time >= 0 and time <= 12:
                weather_now.sun_altitude_angle = -90 + time / 12 * 180
            else:
                weather_now.sun_altitude_angle = 90 - ((time - 12) / 12 * 180)
        for weather in env.get_weathers().get_weathers():
            if weather.get_weather_kind().value == 0:
                # check kind type
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.cloudiness = (
                        1-weather.get_weather_kind().value)*100
                else:
                    if weather.get_weather_kind_value().get_level().value == 0:
                        weather.cloudiness = (1 - light) * 100
                    elif weather.get_weather_kind_value().get_level().value == 1:
                        weather.cloudiness = (1 - middle) * 100
                    else:
                        weather.cloudiness = (1 - heavy) * 100
            elif weather.get_weather_kind().value == 1:
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.precipitation = weather.get_weather_kind().value*100
                    weather.precipitation_deposits = weather.get_weather_kind().value*100
                else:
                    if weather.get_weather_kind_value().get_level().value == 0:
                        weather.precipitation = light * 100
                        weather.precipitation_deposits = light * 100
                    elif weather.get_weather_kind_value().get_level().value == 1:
                        weather.precipitation = middle * 100
                        weather.precipitation_deposits = middle * 100
                    else:
                        weather.precipitation = heavy * 100
                        weather.precipitation_deposits = heavy * 100
            elif weather.get_weather_kind().value == 3:
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.fog_density = weather.get_weather_kind().value * 100
                    weather.fog_distance = weather.get_weather_kind().value * 1000
                else:
                    if weather.get_weather_kind_value().get_level().value == 0:
                        weather.fog_density = light * 100
                        weather.fog_distance = light * 1000
                    elif weather.get_weather_kind_value().get_level().value == 1:
                        weather.fog_density = middle * 100
                        weather.fog_distance = middle * 1000
                    else:
                        weather.fog_density = heavy * 100
                        weather.fog_distance = heavy * 1000
            elif weather.get_weather_kind().value == 4:
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.wetness = weather.get_weather_kind().value * 100
                else:
                    if weather.get_weather_kind_value().get_level().value == 0:
                        weather.wetness = light * 100
                    elif weather.get_weather_kind_value().get_level().value == 1:
                        weather.wetness = middle * 100
                    else:
                        weather.wetness = heavy * 100
                pass
        pass
        self.world.set_weather(weather_now)
    # destory all actors

    def destory(self):
        # npc thread
        if self.npc_thread is not None and self.npc_thread.is_alive():
            utils.stop_thread(self.npc_thread)
        # self.actor_list.reverse()
        actor_ids = [x.id for x in self.actor_list]
        print(actor_ids)
        self.client.apply_batch_sync(
            [carla.command.DestroyActor(x) for x in actor_ids], True)
        self.actor_list = []
        self.autopilot_batch = []
        self.vehicle_agent_dict = {}
        self.npc_thread = None
        self.blueprint_library = None
        self.id_name_map = {}

    # if key_id in id_name_map

    def id_name_map_has(self, key_id):
        # self.id_name_map = {
        #     '1': 'npc1',
        #     '2': 'npc2',
        #     '3': 'npc3',
        #     '4': 'pedestrian1'
        # }
        return key_id in self.id_name_map

    # get corredponding name with id_name_map
    def id_corresponding_name(self, key_id):
        # self.id_name_map = {
        #     '1': 'npc1',
        #     '2': 'npc2',
        #     '3': 'npc3',
        #     '4': 'pedestrian1'
        # }
        assert self.id_name_map_has(key_id)
        return self.id_name_map[key_id]


class NPCControlThread(threading.Thread):
    def __init__(self, vehicle_agent_dict, world):
        threading.Thread.__init__(self)
        self.vehicle_agent_dict = vehicle_agent_dict
        self.world = world

    def run(self):
        for adapted_vehicle in list(self.vehicle_agent_dict.keys()):
            adapted_vehicle.set_speed()
        while True:
            if not self.world.wait_for_tick(10.0):
                continue

            if len(self.vehicle_agent_dict) == 0:
                break

            for adapted_vehicle in list(self.vehicle_agent_dict.keys()):
                agent = self.vehicle_agent_dict[adapted_vehicle]
                agent.update_information(self.world)
                if len(agent.get_local_planner().waypoints_queue) == 0:
                    print("[{}]:reached".format(adapted_vehicle.name))
                    adapted_vehicle.stop()
                    self.vehicle_agent_dict.pop(adapted_vehicle)
                else:
                    control = agent.run_step()
                    adapted_vehicle.carla_actor.apply_control(control)


class AdaptedActor:
    def __init__(self, world, ast_actor, actor_type="Vehicle"):
        self.world = world
        self.ast_actor = ast_actor
        self.carla_actor = None
        self.name = ast_actor.get_name()
        self.start_transform = self.__get_valid_transform(
            self.ast_actor.get_first_state().get_position(), actor_type)
        self.start_transform.location.z = self.start_transform.location.z + 0.3  # avoid collision
        if not not self.ast_actor.get_second_state():
            self.target_transform = self.__get_valid_transform(
                self.ast_actor.get_second_state().get_position(), actor_type)
        else:
            self.target_transform = None
        self.blueprint_library = self.world.get_blueprint_library()
        self.blueprint = self.__get_blueprint()
        self.path_transform_list = self.__get_path_transform_list(actor_type)

    def __get_valid_transform(self, ast_position, actor_type):
        if not ast_position.has_frame() or ast_position.is_frame_ENU():
            if ast_position.is_normal_coordinate():
                location = carla.Location(
                    ast_position.get_coordinate(
                    ).get_x(), ast_position.get_coordinate().get_y(), 0)
                # auto choose the nearest waypoint
                transform = self.__get_waypoint_by_location(
                    location, actor_type).transform
            else:
                transform = self.__get_waypoint_by_mixed_laneid(ast_position.get_coordinate(
                ).get_lane().get_lane_id(), ast_position.get_coordinate().get_distance()).transform
        else:
            location = carla.Location(0, 0, 5)
            transform = self.__get_waypoint_by_location(
                location, actor_type).transform
        return transform

    def __get_waypoint_by_location(self, location, actor_type="Vehicle"):
        if actor_type == "Vehicle":
            waypoint = self.world.get_map().get_waypoint(
                location, lane_type=carla.LaneType.Driving)
        elif actor_type == "Pedestrian":
            waypoint = self.world.get_map().get_waypoint(
                location, lane_type=carla.LaneType.Sidewalk)
        else:
            print("Error:wrong actor type")
            waypoint = self.world.get_map().get_waypoint(location)
        return waypoint

    def __get_waypoint_by_mixed_laneid(self, mixed_lane_id, length):
        road_id, lane_id = mixed_lane_id.split('.')
        assert len(road_id) > 0 and len(lane_id) > 0
        waypoint = self.world.get_map().get_waypoint_xodr(
            int(road_id), int(lane_id), length)
        assert waypoint is not None
        return waypoint

    def __get_blueprint(self):
        return None

    def __get_path_transform_list(self, actor_type):
        path_transform_list = [self.start_transform]
        try:
            if self.ast_actor.has_vehicle_motion():
                middle_transform_list = []
                for state in self.ast_actor.get_vehicle_motion().get_motion().get_state_list().get_states():
                    position = state.get_position()
                    transform = self.__get_valid_transform(
                        position, actor_type)
                    middle_transform_list.append(transform)
                path_transform_list = path_transform_list + middle_transform_list
        except:
            pass
        path_transform_list.append(self.target_transform)
        return path_transform_list

    def spawn(self):
        try:
            actor = self.world.spawn_actor(
                self.blueprint, self.start_transform)
            self.carla_actor = actor
            return actor
        except Exception as exception:
            print("Spawn error:{}".format(exception))
            return None

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
        info_string = "[{}]:({},{},{})->({},{},{})".format(self.ast_actor.get_name(
        ), __get_float_value(self.start_transform.location.x),
            __get_float_value(self.start_transform.location.y),
            __get_float_value(self.start_transform.location.z),
            __get_float_value(self.target_transform.location.x),
            __get_float_value(self.target_transform.location.y),
            __get_float_value(self.target_transform.location.z))
        return info_string

    def distance_to_target(self):
        target_location = self.target_transform.location
        current_location = self.carla_actor.get_location()
        delta_dist = math.sqrt(
            ((target_location.x-current_location.x)**2)+((target_location.y-current_location.y)**2))
        return delta_dist

    def has_reached(self, radius=0.5):
        delta_dist = self.distance_to_target()
        if delta_dist < radius:
            return True
        else:
            return False


class AdaptedVehicle(AdaptedActor):
    def __init__(self, world, ast_actor):
        AdaptedActor.__init__(self, world, ast_actor, "Vehicle")
        self.car = []
        self.bus = []
        self.van = []
        self.truck = []
        self.bicycle = []
        self.motorbicycle = []
        self.__get_types_of_cars()
        self.blueprint = self.__get_blueprint()

    def __get_blueprint(self):
        if self.ast_actor.has_vehicle_type():
            blueprint = random.choice(self.car)
            if self.ast_actor.get_vehicle_type().is_specific_type():
                vehicle_list = self.blueprint_library.filter(
                    self.ast_actor.get_vehicle_type().get_type().get_value())
                if len(vehicle_list) != 0:
                    blueprint = random.choice(vehicle_list)
            else:
                kind = self.ast_actor.get_vehicle_type().get_type().get_type().get_kind().value
                if kind == 0:
                    blueprint = random.choice(self.car)
                elif kind == 1:
                    blueprint = random.choice(self.bus)
                elif kind == 2:
                    blueprint = random.choice(self.van)
                elif kind == 3:
                    blueprint = random.choice(self.truck)
                elif kind == 4:
                    blueprint = random.choice(self.bicycle)
                elif kind == 5:
                    blueprint = random.choice(self.motorbicycle)
            if self.ast_actor.get_vehicle_type().has_color():
                if self.ast_actor.get_vehicle_type().is_rgb_color():
                    if blueprint.has_attribute('color'):
                        # get the color in data
                        color_adapter = self.ast_actor.get_vehicle_type().get_color()
                        # transform into carla.Color
                        color = str(color_adapter.get_r(
                        ))+','+str(color_adapter.get_g())+','+str(color_adapter.get_b())
                    else:
                        color_list_value = self.ast_actor.get_vehicle_type().get_color().get_kind().value
                        if color_list_value == 0:
                            color = '255,0,0'
                        elif color_list_value == 1:
                            color = '0,255,0'
                        elif color_list_value == 2:
                            color = '0,0,255'
                        elif color_list_value == 3:
                            color = '0,0,0'
                        else:
                            color = '255,255,255'
                    blueprint.set_attribute('color', color)
        else:
            blueprint = random.choice(self.car)
        return blueprint

    def __get_types_of_cars(self):
        car_list = ['nissan', 'audi', 'bmw', 'chevrolet', 'citroen', 'dodge_charger', 'wrangler_rubicon',
                    'mercedes-benz',
                    'cooperst', 'seat', 'toyota', 'model3', 'lincoln', 'mustang']
        car_blue_list = []
        #car = []
        for car in car_list:
            car_blue_list.append(self.blueprint_library.filter(car))
        for car_blue in car_blue_list:
            for car in car_blue:
                self.car.append(car)
        #bus = []
        for bp in self.blueprint_library.filter('volkswagen'):
            self.bus.append(bp)
        #van = []
        for bp in self.blueprint_library.filter('carlacola'):
            self.van.append(bp)
        #truck = []
        for bp in self.blueprint_library.filter('cybertruck'):
            self.truck.append(bp)
        bicycle_list = ['crossbike', 'omafiets', 'century']
        #bicycle = []
        for bicycle in bicycle_list:
            for bp in self.blueprint_library.filter(bicycle):
                self.bicycle.append(bp)
        motorbicycle_list = ['harley-davidson', 'ninja', ' yamaha']
        #motorbicycle = []
        for motorbicycle in motorbicycle_list:
            for bp in self.blueprint_library.filter(motorbicycle):
                self.motorbicycle.append(bp)

    def set_speed(self):
        if self.ast_actor.get_first_state().has_speed():
            speed = float(
                self.ast_actor.get_first_state().get_speed().get_speed_value())
            if speed >= 0:
                self.carla_actor.enable_constant_velocity(
                    carla.Vector3D(speed, 0, 0))
                print("set {} speed to {}".format(
                    self.ast_actor.get_name(), speed))

    def use_auto(self):
        if self.ast_actor.get_first_state().has_speed():
            speed = float(
                self.ast_actor.get_first_state().get_speed().get_speed_value())
            if speed >= 0:
                return False
            else:
                return True
        else:
            return False

    def stop(self):
        self.carla_actor.enable_constant_velocity(carla.Vector3D(0, 0, 0))


class AdaptedPedestrian(AdaptedActor):
    def __init__(self, world, ast_actor):
        AdaptedActor.__init__(self, world, ast_actor, "Pedestrian")
        self.pedestrian = []
        self.__get_types_of_pedestrian()
        self.blueprint = self.__get_blueprint()

    def __get_blueprint(self):
        return random.choice(self.pedestrian)

    def __get_types_of_pedestrian(self):
        for bp in self.blueprint_library.filter('pedestrian'):
            self.pedestrian.append(bp)

    def start_ai_walk(self):
        if self.carla_actor is not None:
            walker_controller_bp = self.blueprint_library.find(
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
    def __init__(self, world, ast_actor):
        AdaptedActor.__init__(self, world, ast_actor, "Obstacle")
        self.obstacle_tiny = []
        self.obstacle_small = []
        self.obstacle_medium = []
        self.obstacle_big = []
        self.__get_obstacle()
        self.blueprint = self.__get_blueprint()

    def __get_blueprint(self):
        return random.choice(self.obstacle_big)

    def __get_obstacle(self):
        for bp in self.blueprint_library.filter('static'):
            if bp.has_attribute('size'):
                if 'tiny' in bp.get_attribute('size').as_str():
                    self.obstacle_tiny.append(bp)
                elif 'small' in bp.get_attribute('size').as_str():
                    self.obstacle_small.append(bp)
                elif 'medium' in bp.get_attribute('size').as_str():
                    self.obstacle_medium.append(bp)
                else:
                    self.obstacle_big.append(bp)
