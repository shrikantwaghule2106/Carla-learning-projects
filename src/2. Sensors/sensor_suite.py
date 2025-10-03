import carla

class SensorSuite:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.world = vehicle.get_world()

    def setup_sensors(self):
        print("Sensor suite initialized")

if __name__ == "__main__":
    print("Phase 2: Sensor Systems")
