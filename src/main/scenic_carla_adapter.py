import threading
from src.main.carla_adapter import CarlaAdapter
import scenic
from src.tools import utils

class ScenicCarlaAdapter(CarlaAdapter):
    def __init__(self, ip_address, carla_map):
        super().__init__(ip_address)
        self.params = {}
        self.params["address"] = ip_address
        # TODO: remove
        self.params["carla_map"] = carla_map
        self.scene = None
        self.simulator = None
        self.simulate_thread = None

    def set_map(self, map):
        super().set_map(map)
        self.params["carla_map"] = map

    def init(self, code_file):
        scenario = scenic.scenarioFromFile(path = code_file, model="scenic.simulators.carla.model", params=self.params)
        self.scene,_ = scenario.generate()
        self.simulator = scenario.getSimulator()
        self.simulator.render = False

    def run(self):
        self.simulate_thread = SimulateThread(self.simulator, self.scene)
        self.simulate_thread.start()

    def stop(self):
        if self.simulate_thread is not None:
            self.simulate_thread.stop()
            # self.simulate_thread.join()
            utils.stop_thread(self.simulate_thread)
            # super().stop()
        self.simulate_thread = None

class SimulateThread(threading.Thread):
    def __init__(self, simulator, scene):
        threading.Thread.__init__(self)
        self.simulator = simulator
        self.scene = scene

    def run(self):
        try:
            self.simulator.simulate(self.scene, verbosity=0)
        except Exception as exception:
            print("Scenic error:{}".format(exception))
            return 

    def stop(self):
        try:
            print("Destroying scenic simulator")
            self.simulator.destroy()
        except Exception as exception:
            print("Destroy scenic simulator error:{}".format(exception))