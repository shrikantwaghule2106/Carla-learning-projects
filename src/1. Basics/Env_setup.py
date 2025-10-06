import carla
import pygame
import numpy as np
import random

def spawn_vehicle(world, blueprint_library):
    try:
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        spawn_points = world.get_map().get_spawn_points()
        if spawn_points:
            spawn_point = random.choice(spawn_points)
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
            return vehicle
        else:
            print("No spawn points available")
            return None

        #Choose a random spawn point
        spawn_point = random.choice(spawn_points)

        # Spawn the vehicle
        # vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        # print(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")
        # return vehicle
        #
    except Exception as e:
        print(f"Error spawning vehicle: {e}")
        return None

def main():
    print("CARLA Environment Setup")

    # Initialize variables for cleanup
    vehicle = None
    camera = None

    try:
        # Connect to the client
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Get the world
        world = client.get_world()
        print(f"Connected to world: {world}")
        print(f"Map: {world.get_map().name}")
        print(f"All actors: {len(world.get_actors())}")
        print(f"All blueprints: {len(world.get_blueprint_library().filter('*'))}")

        # Display weather information
        weather = world.get_weather()
        print(f"Current weather: Cloudiness={weather.cloudiness}, Precipitation={weather.precipitation}")

        # Display available maps
        available_maps = client.get_available_maps()
        print(f"Available maps: {len(available_maps)}")

        # Test vehicle spawning
        print("\n--- Testing Vehicle Spawning ---")
        blueprint_library = world.get_blueprint_library()
        vehicle = spawn_vehicle(world, blueprint_library)

        # Initialize pygame for camera display
        pygame.init()
        display = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("CARLA Camera View")

        # Add camera to the spawned vehicle
        if vehicle:
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '800')
            camera_bp.set_attribute('image_size_y', '600')
            camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

            # Camera callback function
            def camera_callback(image):
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                array = array[:, :, :3]
                array = array[:, :, ::-1]
                surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                display.blit(surface, (0, 0))
                pygame.display.flip()

            camera.listen(camera_callback)
            print(f"✓ Camera attached and displaying")

        # Make vehicle move
        if vehicle:
            vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=0.0))
            print("✓ Vehicle set to move forward")

        # Main game loop
        clock = pygame.time.Clock()
        running = True
        start_time = pygame.time.get_ticks()

        print("Running simulation for 10 seconds...")
        while running:
            # Run for 10 seconds
            if pygame.time.get_ticks() - start_time > 10000:  # 10 seconds
                running = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            clock.tick(30)  # 30 FPS

    finally:
        # Cleanup
        print("\nCleaning up...")
        if camera is not None:
            camera.stop()
            camera.destroy()
        if vehicle is not None:
            vehicle.destroy()
        pygame.quit()
        print("Cleanup completed")
if __name__ == "__main__":
    main()
