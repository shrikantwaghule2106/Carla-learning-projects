import carla
import numpy as np
import pygame
import random
from collections import deque

class SensorManager:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.sensors = []
        self.data_queues = {}

    def add_camera(self, blueprint_library, display,  position=carla.Location(x=1.5, z=2.4)):
        try:
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '800')
            camera_bp.set_attribute('image_size_y', '600')
            camera_bp.set_attribute('fov', '110')

            camera_transform = carla.Transform(position)
            camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)

          # Create a queue to store the images
            self.data_queues['camera'] = deque(maxlen=1)

          # Define the callback function to process images
            def camera_callback(image):
                array = np.frombuffer(image.raw_data, dtype=np.uint8)
                array = array.reshape((image.height, image.width, 4))
                array = array[:,:,:3]
                array = array[:, :, ::-1]  # Convert BGRA to RGB
                self.data_queues['camera'].append(array)

        # Display if display surafce is provided
                if display:
                    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                    display.blit(surface, (0, 0))
                    pygame.display.flip()

            camera.listen(camera_callback)
            self.sensors.append(camera)
            print("✓ Camera sensor added")
            return camera
        except Exception as e:
            print(f"Error adding camera sensor: {e}")
            return None
    def cleanup(self):
        for sensor in self.sensors:
            sensor.stop()
            sensor.destroy()
    def add_lidar(self, blueprint_library):
        try:
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '50')
            lidar_bp.set_attribute('rotation_frequency', '10')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('points_per_second', '56000')

            lidar_transform = carla.Transform(carla.Location(z=2.5))
            lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)

            self.data_queues['lidar'] = deque(maxlen=1)

            def lidar_callback(point_cloud):
                points = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
                points = np.reshape(points, (int(points.shape[0] / 4), 4))
                self.data_queues['lidar'].append(points)

            lidar.listen(lidar_callback)
            self.sensors.append(lidar)
            print("✓ LIDAR sensor added")
            return lidar
        except Exception as e:
            print(f"Error adding LIDAR sensor: {e}")
            return None
    def add_collision_sensor(self, blueprint_library):
        try:
            collision_bp = blueprint_library.find('sensor.other.collision')
            collision_transform = carla.Transform()
            collision_sensor = self.world.spawn_actor(collision_bp, collision_transform, attach_to=self.vehicle)

            self.data_queues['collision'] = deque(maxlen=10)

            def collision_callback(event):
                self.data_queues['collision'].append(event)
                print("Collision detected!")

            collision_sensor.listen(collision_callback)
            self.sensors.append(collision_sensor)
            print("✓ Collision sensor added")
            return collision_sensor
        except Exception as e:
            print(f"Error adding collision sensor: {e}")
            return None

def spawn_vehicle(world, blueprint_library):
    try:
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        spawn_points = world.get_map().get_spawn_points()
        if spawn_points:
            spawn_point = random.choice(spawn_points)
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            print(f"✓ Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
            return vehicle
        else:
            print("No spawn points available")
            return None
    except Exception as e:
        print(f"Error spawning vehicle: {e}")
        return None

def main():
    print("CARLA Sensor Integration")

    #Initialize variables
    vehicle = None
    sensor_manager = None

    try:
        #Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        #Set up pygame display
        pygame.init()
        display = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("CARLA Sensor Integration")

        #spwan vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle = spawn_vehicle(world, blueprint_library)
        if vehicle:
            #Initialize Sensor Manager
            sensor_manager = SensorManager(world, vehicle)
            #Add camera sensor
            sensor_manager.add_camera(blueprint_library, display)
            #Lidar sensor
            sensor_manager.add_lidar(blueprint_library)
            sensor_manager.add_collision_sensor(blueprint_library)
            #Make vehicle move
            vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer = 0.0))
        #Main loop
            clock = pygame.time.Clock()
            running = True
            start_time = pygame.time.get_ticks()
            while running:
                if pygame.time.get_ticks() - start_time > 15000:  # Run for 15 seconds
                    running = False
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                clock.tick(30)
    except Exception as e:
        print(f"Error in main loop: {e}")

    finally:
        print("Cleaning up...")
        if sensor_manager:
            sensor_manager.cleanup()
        if vehicle:
            vehicle.destroy()
        pygame.quit()
        print("Done.")

if __name__ == "__main__":
    print("Phase 2: Sensor Systems")
    main()
