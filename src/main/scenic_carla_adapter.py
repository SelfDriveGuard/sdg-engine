from src.main.carla_adapter import CarlaAdapter

class ScenicCarlaAdapter(CarlaAdapter):
    def __init__(self, ip_address):
        super().__init__(ip_address)
        print("Scenic adapter created")