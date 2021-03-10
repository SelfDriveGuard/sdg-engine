import carla
import random
import math

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