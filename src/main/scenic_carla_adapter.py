import threading
from src.main.carla_adapter import CarlaAdapter
import scenic
from src.tools import utils


class ScenicCarlaAdapter(CarlaAdapter):
    def __init__(self, ip_address):
        super().__init__(ip_address)
        self.scene = None
        self.simulator = None
        self.simulate_thread = None
        self.callback = None

    def init(self, scenario, map_name, callback):
        self.scene, _ = scenario.generate()
        # set map before simulator is created
        if not map_name:
            super().set_map(
                self.scene.params["carla_map"])
        else:
            super().set_map(map_name)
        # TODO: handle simulator init error
        self.simulator = scenario.getSimulator()
        self.simulator.render = False
        self.callback = callback

        # TODO: whether need to wait
        # if not self.world.wait_for_tick(100.0):
        #     print("Map load failed")
        super().set_spectator()

    def get_av_ego(self):
        for car_object in list(self.scene.objects):
            if car_object.rolename == "AV_EGO":
                temp_list = list(self.scene.objects)
                temp_list.remove(car_object)
                self.scene.objects = tuple(temp_list)
                return car_object
        return None

    def run(self):
        # debug
        self.show_info()
        self.simulate_thread = SimulateThread(self.simulator, self.scene, self.callback)
        self.simulate_thread.start()

    def stop(self):
        try:
            if self.simulate_thread is not None:
                self.simulate_thread.stop()
                # self.simulate_thread.join()
                utils.stop_thread(self.simulate_thread)
        except Exception as exception:
            print("Stop simulator thread error:{}".format(exception))
        finally:
            super().stop()
            self.simulate_thread = None

    def show_info(self):
        for obj in self.scene.objects:
            print("{}:[{}]@{}".format(obj, obj.rolename, obj.position.coordinates))


class SimulateThread(threading.Thread):
    def __init__(self, simulator, scene, callback):
        threading.Thread.__init__(self)
        self.simulator = simulator
        self.scene = scene
        self.callback = callback

    def run(self):
        try:
            self.simulator.simulate(self.scene, verbosity=0)
        except Exception as exception:
            self.callback(cmd="ERROR", msg="Scenic error: {}".format(exception))
            print("Scenic error: {}".format(exception))

    def stop(self):
        try:
            print("Destroying scenic simulator")
            self.simulator.destroy()
        except Exception as exception:
            print("Destroy scenic simulator error:{}".format(exception))
