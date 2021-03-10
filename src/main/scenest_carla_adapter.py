from src.main.carla_adapter import CarlaAdapter, AdaptedVehicle, AdaptedPedestrian, AdaptedObstacle
import threading
import carla
from src.scenest_parser.ast.base.weathers import WeatherContinuousIndex
from src.tools import utils
from src.tools.agents.navigation.behavior_agent import BehaviorAgent

class ScenestCarlaAdapter(CarlaAdapter):
    def __init__(self, ip_address):
        super().__init__(ip_address)
        self.scenario = None
        self.actor_list = []
        self.autopilot_batch = []
        self.vehicle_agent_dict = {}
        self.npc_thread = None
        self.id_name_map = {}
        self.traffic_manager = self.client.get_trafficmanager(9988)

    def init(self, scenario):
        self.scenario = scenario
        if self.scenario.has_npc_vehicles():
            self.__create_npc_vehicles(self.scenario.get_npc_vehicles())

    def run(self):
        if self.scenario.has_npc_vehicles():
            self.__run_npc_vehicles()
        if self.scenario.has_pedestrians():
            self.__set_pedestrians(self.scenario.get_pedestrians())
        if self.scenario.has_obstacles():
            self.__set_obstacles(self.scenario.get_obstacles())
        if self.scenario.has_environment():
            self.__set_environment(self.scenario.get_environment())

    def stop(self):
        # npc thread
        if self.npc_thread is not None and self.npc_thread.is_alive():
            utils.stop_thread(self.npc_thread)
        actor_ids = [x.id for x in self.actor_list]
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in actor_ids])
        self.actor_list = []
        self.autopilot_batch = []
        self.vehicle_agent_dict = {}
        self.npc_thread = None
        self.id_name_map = {}
        super().stop()

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


    def __create_npc_vehicles(self, npcs):
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
                # set speed
                adapted_vehicle.set_speed()
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
                adapted_pedestrian = AdaptedPedestrian(self.world, pedestrian)
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
                adapted_obstacle = AdaptedObstacle(self.world, obstacle)
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

class NPCControlThread(threading.Thread):
    def __init__(self, vehicle_agent_dict, world):
        threading.Thread.__init__(self)
        self.vehicle_agent_dict = vehicle_agent_dict
        self.world = world

    def run(self):
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