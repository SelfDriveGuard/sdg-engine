from src.main.carla_adapter import CarlaAdapter, AdaptedVehicle, AdaptedPedestrian, AdaptedObstacle
import threading
import carla
from src.scenest_parser.ast.base.weathers import WeatherContinuousIndex
from src.tools import utils
from src.tools.agents.navigation.behavior_agent import BehaviorAgent
import random


class ScenestCarlaAdapter(CarlaAdapter):
    def __init__(self, ip_address):
        super().__init__(ip_address)
        self.scenario = None
        self.autopilot_batch = []
        self.vehicle_agent_dict = {}
        self.npc_thread = None
        self.id_name_map = {}
        self.traffic_manager = self.client.get_trafficmanager(9988)
        self.traffic_manager.set_hybrid_physics_mode(True)

    def init(self, scenario):
        self.scenario = scenario
        if self.scenario.has_npc_vehicles():
            self.__create_npc_vehicles(self.scenario.get_npc_vehicles())
        if self.scenario.has_environment():
            self.__set_environment(self.scenario.get_environment())

    def run(self):
        if self.scenario.has_npc_vehicles():
            self.__run_npc_vehicles()
        if self.scenario.has_pedestrians():
            self.__set_pedestrians(self.scenario.get_pedestrians())
        if self.scenario.has_obstacles():
            self.__set_obstacles(self.scenario.get_obstacles())

    def stop(self):
        # npc thread
        if self.npc_thread is not None and self.npc_thread.is_alive():
            utils.stop_thread(self.npc_thread)
        self.autopilot_batch = []
        self.vehicle_agent_dict = {}
        self.npc_thread = None
        self.id_name_map = {}
        super().stop()

    def __create_npc_vehicles(self, npcs):
        print("Number of NPCs:"+str(npcs.get_size()))
        if npcs.get_size() < 1:
            return
        spawn_batch = []
        adapted_vehicles = []

        npc_vehicles = npcs._vehicles
        for npc in npc_vehicles:
            if npc.get_first_state().has_speed():
                speed = npc.get_first_state().get_speed().get_speed_value()
            else:
                speed = None
            adapted_vehicle = AdaptedVehicle(
                self.world, npc.get_name(), self.__get_vehicle_blueprint(npc), speed)
            set_position(npc, adapted_vehicle)
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

    def __run_npc_vehicles(self):
        for response in self.client.apply_batch_sync(self.autopilot_batch, True):
            if response.error:
                print(response.error)
        self.npc_thread = NPCControlThread(self.vehicle_agent_dict, self.world)
        self.npc_thread.start()

    def __set_pedestrians(self, peds):
        print("Number of pedestrians:"+str(peds.get_size()))
        if peds.get_size() > 0:
            pedestrians = peds.get_pedestrians()
            for pedestrian in pedestrians:
                adapted_pedestrian = AdaptedPedestrian(
                    self.world, pedestrian.get_name(), self.__get_pedestrian_blueprint())
                set_position(pedestrian, adapted_pedestrian)
                adapted_pedestrian.spawn()
                # 维护walker_name变量名与carla中actor.id的对应关系
                self.id_name_map[str(
                    adapted_pedestrian.carla_actor.id)] = adapted_pedestrian.name

                self.actor_list.append(adapted_pedestrian.carla_actor)
                adapted_pedestrian.start_ai_walk()
                print(adapted_pedestrian.info())

    def __set_obstacles(self, obs):
        print("Number of obstacles:"+str(obs.get_size()))
        obstacles = obs.get_obstacle()
        if obstacles.get_size() > 0:
            for obstacle in obstacles:
                adapted_obstacle = AdaptedObstacle(
                    self.world, self.__get_obstacle_blueprint())
                set_position(obstacle, adapted_obstacle)
                obstacle = adapted_obstacle.spawn()
                print(adapted_obstacle.info())
                self.actor_list.append(obstacle)

    def __set_environment(self, env):
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
                        1-weather.get_weather_kind_value().get_index())*100
                else:
                    if weather.get_weather_kind_value().get_level().value == 0:
                        weather.cloudiness = (1 - light) * 100
                    elif weather.get_weather_kind_value().get_level().value == 1:
                        weather.cloudiness = (1 - middle) * 100
                    else:
                        weather.cloudiness = (1 - heavy) * 100
                weather_now.precipitation = 0
                weather_now.precipitation_deposits = 0
                weather_now.cloudiness = weather.cloudiness
            elif weather.get_weather_kind().value == 1:
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.precipitation = weather.get_weather_kind_value().get_index()*100
                    weather.precipitation_deposits = weather.get_weather_kind_value().get_index()*100
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
                weather_now.precipitation = weather.precipitation
                weather_now.precipitation_deposits = weather.precipitation_deposits
            elif weather.get_weather_kind().value == 3:
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.fog_density = weather.get_weather_kind_value().get_index() * 100
                    weather.fog_distance = weather.get_weather_kind_value().get_index() * 1000
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
                weather_now.fog_density = weather.fog_density
                weather_now.fog_distance = weather.fog_distance
            elif weather.get_weather_kind().value == 4:
                if type(weather.get_weather_kind_value()) == WeatherContinuousIndex:
                    weather.wetness = weather.get_weather_kind_value().get_index() * 100
                else:
                    if weather.get_weather_kind_value().get_level().value == 0:
                        weather.wetness = light * 100
                    elif weather.get_weather_kind_value().get_level().value == 1:
                        weather.wetness = middle * 100
                    else:
                        weather.wetness = heavy * 100
                weather_now.wetness = weather.wetness
                pass
        pass
        print(weather_now)
        self.world.set_weather(weather_now)

    def __get_vehicle_blueprint(self, npc_ast):
        car_list = ['nissan', 'audi', 'bmw', 'chevrolet', 'citroen', 'dodge_charger', 'wrangler_rubicon',
                    'mercedes-benz',
                    'cooperst', 'seat', 'toyota', 'model3', 'lincoln', 'mustang']
        car_blue_list = []
        cars = []
        for car in car_list:
            car_blue_list.append(self.blueprint_library.filter(car))
        for car_blue in car_blue_list:
            for car in car_blue:
                cars.append(car)
        buses = []
        for bp in self.blueprint_library.filter('volkswagen'):
            buses.append(bp)
        vans = []
        for bp in self.blueprint_library.filter('carlacola'):
            vans.append(bp)
        trucks = []
        for bp in self.blueprint_library.filter('cybers'):
            trucks.append(bp)
        bicycle_list = ['crossbike', 'omafiets', 'century']
        bicycles = []
        for bicycle in bicycle_list:
            for bp in self.blueprint_library.filter(bicycle):
                bicycles.append(bp)
        motorbicycle_list = ['harley-davidson', 'ninja', ' yamaha']
        motorbicycles = []
        for motorbicycle in motorbicycle_list:
            for bp in self.blueprint_library.filter(motorbicycle):
                motorbicycles.append(bp)

        if npc_ast.has_vehicle_type():
            blueprint = random.choice(cars)
            if npc_ast.get_vehicle_type().is_specific_type():
                vehicle_list = self.blueprint_library.filter(
                    npc_ast.get_vehicle_type().get_type().get_value())
                if len(vehicle_list) != 0:
                    blueprint = random.choice(vehicle_list)
            else:
                kind = npc_ast.get_vehicle_type().get_type().get_type().get_kind().value
                if kind == 0:
                    blueprint = random.choice(cars)
                elif kind == 1:
                    blueprint = random.choice(buses)
                elif kind == 2:
                    blueprint = random.choice(vans)
                elif kind == 3:
                    blueprint = random.choice(trucks)
                elif kind == 4:
                    blueprint = random.choice(bicycles)
                elif kind == 5:
                    blueprint = random.choice(motorbicycles)
            if npc_ast.get_vehicle_type().has_color():
                if npc_ast.get_vehicle_type().is_rgb_color():
                    if blueprint.has_attribute('color'):
                        # get the color in data
                        color_adapter = npc_ast.get_vehicle_type().get_color()
                        # transform into carla.Color
                        color = str(color_adapter.get_r(
                        ))+','+str(color_adapter.get_g())+','+str(color_adapter.get_b())
                    else:
                        color_list_value = npc_ast.get_vehicle_type().get_color().get_kind().value
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
            blueprint = random.choice(cars)
        return blueprint

    def __get_pedestrian_blueprint(self):
        pedestrian = []
        for bp in self.blueprint_library.filter('pedestrian'):
            pedestrian.append(bp)
        return random.choice(pedestrian)

    def __get_obstacle_blueprint(self):
        obstacle_tiny = []
        obstacle_small = []
        obstacle_medium = []
        obstacle_big = []
        for bp in self.blueprint_library.filter('static'):
            if bp.has_attribute('size'):
                if 'tiny' in bp.get_attribute('size').as_str():
                    obstacle_tiny.append(bp)
                elif 'small' in bp.get_attribute('size').as_str():
                    obstacle_small.append(bp)
                elif 'medium' in bp.get_attribute('size').as_str():
                    obstacle_medium.append(bp)
                else:
                    obstacle_big.append(bp)
        return random.choice(obstacle_big)

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


def get_position_and_type(ast_position):
    if ast_position.is_normal_coordinate():
        position_type = "COORDINATE"
        position_value = (ast_position.get_coordinate().get_x(),
                          ast_position.get_coordinate().get_y())
    else:
        road_id, lane_id = ast_position.get_coordinate().get_lane().get_lane_id().split('.')
        length = ast_position.get_coordinate().get_distance()
        position_type = "LANE"
        position_value = (road_id, lane_id, length)
    return position_value, position_type


def set_position(ast_actor, adapted_actor):
    # start
    position_value, position_type = get_position_and_type(
        ast_actor.get_first_state().get_position())
    adapted_actor.set_start_position(position_value, position_type)

    # target
    if not not ast_actor.get_second_state():
        position_value, position_type = get_position_and_type(
            ast_actor.get_second_state().get_position())
        adapted_actor.set_target_position(position_value, position_type)

    # middle
    try:
        if ast_actor.has_vehicle_motion():
            middle_positions = []
            for state in ast_actor.get_vehicle_motion().get_motion().get_state_list().get_states():
                position = state.get_position()
                position_value, position_type = get_position_and_type(position)
                middle_positions.append(position_value)
            adapted_actor.set_middle_positions(middle_positions, position_type)
    except:
        pass
