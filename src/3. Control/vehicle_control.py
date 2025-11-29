import carla
import random
import pygame
import numpy as np
from collections import deque
import math
import os
import sys

carla_root = 'D:/Softwares/CARLA/CARLA_0.9.13/WindowsNoEditor'
pythonapi_path = os.path.join(carla_root, 'PythonAPI', 'carla')

if os.path.exists(pythonapi_path):
    sys.path.append(pythonapi_path)
    print("✓ CARLA PythonAPI path added")
else:
    print(f"CARLA PythonAPI not found at: {pythonapi_path}")


class SensorManager:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.sensors = []
        self.data_queues = {}

    def setup_sensors(self, blueprint_library, display,position=carla.Location(x=1.5, z=2.4)):
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
                try:
                    array = array.reshape((image.height, image.width, 4))
                except Exception:
                    return
                array = array[:,:,:3]
                array = array[:, :, ::-1]  # Convert BGRA to RGB
                self.data_queues['camera'].append(array)

        # Display if display surafce is provided
                if display:
                    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
                    display.blit(surface, (0, 0))

            camera.listen(camera_callback)
            self.sensors.append(camera)
            print("✓ Camera sensor added")
            return camera
        except Exception as e:
            print(f"Error adding camera sensor: {e}")
            return None

    def add_lidar(self, blueprint_library):
        try:
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '50')
            lidar_bp.set_attribute('rotation_frequency', '10')
            lidar_bp.set_attribute('channels', '32')
            lidar_bp.set_attribute('points_per_second', '50000')

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
            print("Collision sensor added")
            return collision_sensor
        except Exception as e:
            print(f"Error adding collision sensor: {e}")
            return None

    def check_obstacles(self, max_distance=15.0):
        if 'lidar' not in self.data_queues or not self.data_queues['lidar']:
            return False

        points = self.data_queues['lidar'][0]
        if points is None or len(points) == 0:
            return False

        front_points = points[(points[:,0] > 0) & (points[:,0] < max_distance) & (np.abs(points[:,1]) < 2.0)]

        return len(front_points) > 50

    def cleanup(self):
        for sensor in list(self.sensors):
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:
                pass
        self.sensors.clear()


class PIDController:
    def __init__(self, kp, ki, kd):
        self.ki = ki
        self.kp = kp
        self.kd = kd
        self.integral = 0.0
        self.previous_error = 0.0

    def commute(self, error, dt):

        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class VehicleController:
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.map = world.get_map()

        #PID Controllers
        self.speed_pid = PIDController(0.3, 0.01, 0.05)
        self.steering_pid = PIDController(0.7, 0.001, 0.1)
        self.last_steering = 0.0

        #Navigation
        self.current_waypoint = None
        self.waypoints = []
        self.target_speed = 30.0

        #Vehicle physics
        self.vehicle.set_autopilot(False)

        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.safe_distance = 15.0
        self.emergency_brake = False

        self.traffic_light_state = None
        self.traffic_light_distance = float('inf')
        self.debug_info = {
            'current_speed': 0.0,
            'target_speed': self.target_speed,
            'distance_to_waypoint': 0.0,
            'obstacle_distance': float('inf'),
            'traffic_light_state': 'None'
        }

    def set_destination(self, location):
        try:
            #Get start and end waypoints
            start_waypoint = self.map.get_waypoint(self.vehicle.get_location())
            end_waypoint = self.map.get_waypoint(location)

            #Generate route
            self.waypoints = self.generate_route(start_waypoint, end_waypoint)
            self.current_waypoint = 0
            print(f"Route generated with {len(self.waypoints)} waypoints.")
        except Exception as e:
            print(f"Error setting destination: {e}")


    def generate_route(self, start_waypoint, end_waypoint, sampling_resolution=2.0):
        try:
           grp = self.map.get_waypoint(start_waypoint.transform.location, project_to_road=True)
           route = []
            #Demo Nav
           current = start_waypoint
           max_waypoints = 100

           while current and len(route) < max_waypoints:
               route.append(current)
               # Get next waypoints
               next_waypoints = current.next(sampling_resolution)
               if next_waypoints:
                   # Choose the waypoint closest to the end location
                   current = next_waypoints[0]
               else:
                   break

           return route
        except Exception as e:
            # Fall back to sampling method if planner not available
            print(f"GlobalRoutePlanner unavailable or failed: {e}. Falling back to sampling.")
        return []

    def update_lidar_data_array(self, points):
        try:
            if points is None or len(points) == 0:
                self.obstacle_detected = False
                self.obstacle_distance = float('inf')
                self.emergency_brake = False
                return

            front_points = points[(points[:,0] > 0) & (points[:,0] < self.safe_distance)]
            front_points = front_points[(front_points[:,1] > -2.5) & (front_points[:,1] < 2.5)]
            front_points = front_points[front_points[:,2] > -2.0]
            if len(front_points) > 8:
                min_distance = np.min(front_points[:,0])
                self.obstacle_distance = min_distance
                self.obstacle_detected = min_distance < self.safe_distance
                self.emergency_brake = min_distance < 3.0
            else:
                self.obstacle_detected = False
                self.obstacle_distance = float('inf')
                self.emergency_brake = False

        except Exception as e:
            print(f"Error processing LIDAR array: {e}")

    def update_traffic_light(self):
        try:
            vehicle_location = self.vehicle.get_location()
            vehicle_waypoint = self.map.get_waypoint(vehicle_location, lane_type = carla.LaneType.Driving)
            vehicle_forward = self.vehicle.get_transform().get_forward_vector()

            if not vehicle_forward or not vehicle_waypoint:
                self.traffic_light_state = None
                self.traffic_light_distance = float('inf')
                return

            traffic_lights = self.world.get_actors().filter('traffic.traffic_light*')
            best_distance = float('inf')
            best_state = None

            for traffic_light in traffic_lights:
                try:
                    tl_loc = traffic_light.get_transform().location
                    distance = vehicle_location.distance(tl_loc)

                    if distance > 50.0:
                        continue

                    tl_waypoint = self.map.get_waypoint(tl_loc, lane_type = carla.LaneType.Driving)
                    if not tl_waypoint:
                        continue

                    if tl_waypoint.road_id != vehicle_waypoint.road_id:
                        continue

                    vec_x = tl_loc.x - vehicle_location.x
                    vec_y = tl_loc.y - vehicle_location.y
                    vec_z = tl_loc.z - vehicle_location.z

                    vec_length = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
                    if vec_length < 1e-6:
                        continue

                    vec_x /= vec_length
                    vec_y /= vec_length
                    vec_z /= vec_length

                    dot = vec_x * vehicle_forward.x + vec_y * vehicle_forward.y + vec_z * vehicle_forward.z

                    # light is ahead and reasonably close
                    if dot > 0.5 and distance < 50.0:
                        if distance < best_distance:
                            best_distance = distance
                            best_state = traffic_light.get_state()
                            print(f"Traffic light detected directly ahead at distance {distance:.2f} with state {best_state}")
                except Exception as e:
                    continue

            if best_state is not None:
                self.traffic_light_state = best_state
                self.traffic_light_distance = best_distance
            else:
                self.traffic_light_state = None
                self.traffic_light_distance = float('inf')
        except Exception as e:
            print(f"Error updating traffic light info: {e}")
            self.traffic_light_state = None
            self.traffic_light_distance = float('inf')

    def get_next_waypoint(self):
        if not self.waypoints or self.current_waypoint >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint]

    def calculate_steering(self, current_waypoint):
        if not current_waypoint:
            return 0.0

        #Get vehicle transform
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation

        # Vector to target in world coordinates
        waypoint_location = current_waypoint.transform.location
        dx = waypoint_location.x - vehicle_location.x
        dy = waypoint_location.y - vehicle_location.y

        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = (target_angle - vehicle_rotation.yaw + 180) % 360 - 180

        if abs(angle_diff) < 1.5:
            steering_output = 0.0
        else:
            steering_output = np.clip(angle_diff / 90.0, -1.0, 1.0)

        smoothing_factor = 0.7
        steering = (smoothing_factor * self.last_steering) + ((1 - smoothing_factor) * steering_output)
        self.last_steering = steering

        return steering


    def calculate_throttle_brake(self, current_speed):
        #Adjusted target speed based on conditions
        speed_error = self.target_speed - current_speed

        current_steering = abs(self.last_steering)
        if abs(current_steering) > 0.5:
            adjusted_target = self.target_speed * 0.6
            speed_error = adjusted_target - current_speed

        if speed_error > 0:
            throttle = np.clip(speed_error / self.target_speed, 0.0, 0.7)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-speed_error / self.target_speed, 0.0, 0.5)

        return throttle, brake

    def update(self, dt):

        if self.is_route_complete():
            print("Route complete. Stopping vehicle.")
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=False, reverse=False)
            self.vehicle.apply_control(control)
            return
        if not self.waypoints or self.current_waypoint >= len(self.waypoints):
            return

        current_waypoint = self.get_next_waypoint()
        if not current_waypoint:
            return

        #Get current speed
        velocity = self.vehicle.get_velocity()
        current_speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # km/h

        distance_to_wp = self.vehicle.get_location().distance(current_waypoint.transform.location)

        if  distance_to_wp < 2.0:
            self.current_waypoint += 1
            print(f"Reached waypoint {self.current_waypoint}, moving to next.")
            current_waypoint = self.get_next_waypoint()
            if not current_waypoint:
                return
            distance_to_wp = self.vehicle.get_location().distance(current_waypoint.transform.location)

        #Calculate controls
        steering = self.calculate_steering(current_waypoint)
        throttle, brake = self.calculate_throttle_brake(current_speed)

        self.update_traffic_light()

        if self.traffic_light_state == carla.TrafficLightState.Red and self.traffic_light_distance < 10.0:
            print("Red traffic light ahead, applying brake.")
            throttle = 0.0
            brake = 0.8

        #Apply control
        control = carla.VehicleControl(throttle=throttle, steer=steering, brake=brake, hand_brake=False, reverse=False)
        self.vehicle.apply_control(control)

        self.debug_info.update({
            'current_speed': float(current_speed),
            'target_speed': float(self.target_speed),
            'distance_to_waypoint': float(distance_to_wp),
            'obstacle_distance': float(self.obstacle_distance) if not math.isinf(self.obstacle_distance) else float('inf'),
            'traffic_light_state': str(self.traffic_light_state) if self.traffic_light_state else 'None'
        })

    def is_route_complete(self):
        if not self.waypoints:
            return True
        return self.current_waypoint >= len(self.waypoints)

    def get_debug_info(self):
        return self.debug_info


class AutonomousVehicleSystem:

    def __init__(self, vehicle, world, sensor_manager):
        self.world = world
        self.vehicle = vehicle
        self.sensor_manager = sensor_manager
        self.controller = VehicleController(vehicle, world)
        #Display
        self.display = None
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 28)
        self.last_time = pygame.time.get_ticks()

        #Set random destination
        self.set_forward_lane_destination(forward_distance=40.0)

    def set_forward_lane_destination(self, forward_distance=40.0):
        try:
            vehicle_location = self.vehicle.get_location()
            spawn_wp = self.world.get_map().get_waypoint(vehicle_location, lane_type=carla.LaneType.Driving)
            if not spawn_wp:
                print("Could not find driving lane waypoint for spawn location.")
                return
            # Step forward in small increments to stay on the same lane
            current_wp = spawn_wp
            total_distance = 0.0
            step = 2.0  # meters
            while total_distance < forward_distance:
                next_wps = current_wp.next(step)
                if not next_wps:
                    break
                # Pick the next waypoint that is on the same lane and road
                next_wp = None
                for wp in next_wps:
                    if wp.lane_id == current_wp.lane_id and wp.road_id == current_wp.road_id:
                        next_wp = wp
                        break
                if not next_wp:
                    break
                current_wp = next_wp
                total_distance += step

            destination_wp = current_wp
            self.controller.set_destination(destination_wp.transform.location)
            print(f"Destination set ahead on same lane: {destination_wp.transform.location}")
        except Exception as e:
            print(f"Error setting forward lane destination: {e}")

    def draw_debug_info(self):
        if 'camera' not in self.sensor_manager.data_queues or not self.sensor_manager.data_queues['camera']:
            self.display.fill((0, 0, 0))

        debug_info = self.controller.get_debug_info()
        y_offset = 10

        for key, value in debug_info.items():
            #text = f"{key}: {value:.2f}" if isinstance(value, float) else f"{key}: {value}"
            if isinstance(value, float):
                if math.isinf(value):
                    text = f"{key}: Inf"
                else:
                    text = f"{key}: {value:.2f}"
            else:
                text = f"{key}: {value}"
            text_surface = self.font.render(text, True, (255, 255, 255))
            pygame.draw.rect(self.display, (0, 0, 0), (8, y_offset - 4, text_surface.get_width() + 12, 24))
            self.display.blit(text_surface, (12, y_offset))
            y_offset += 26

        status = "Route Complete" if self.controller.is_route_complete() else "Navigating"
        status_surface = self.font.render(status, True, (0, 255, 0) if status == "Route Complete" else (255, 0, 0))
        pygame.draw.rect(self.display, (0, 0, 0), (8, y_offset + 6, status_surface.get_width() + 12, 28))
        self.display.blit(status_surface, (12, y_offset + 10))
        pygame.display.flip()

    def run(self):
        try:
            running = True
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_r:
                            self.set_forward_lane_destination(forward_distance=40.0)
                            # self.set_random_destination()
                        elif event.key == pygame.K_ESCAPE:
                            running = False
                self.world.tick()

                # Calculate dt (time delta in seconds)
                current_time = pygame.time.get_ticks()
                dt = (current_time - self.last_time) / 1000.0  # Convert milliseconds to seconds
                self.last_time = current_time

                if 'lidar' in self.sensor_manager.data_queues and self.sensor_manager.data_queues['lidar']:
                     lidar_data = self.sensor_manager.data_queues['lidar'][0]
                     self.controller.update_lidar_data_array(lidar_data)

                #Update vehicle controller
                self.controller.update(dt)

                #Draw debug info
                self.draw_debug_info()

                if self.controller.is_route_complete():
                    print("Destination reached. Stopping simulation.")
                    import time
                    time.sleep(2)
                    running = False

                self.clock.tick(30)  # Limit to 30 FPS

        except KeyboardInterrupt:
            print("Simulation interrupted by user.")
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        print("Cleaning up Autonomous Vehicle System...")
        try:
            if self.sensor_manager:
                self.sensor_manager.cleanup()
            if self.vehicle:
                self.vehicle.destroy()
        except Exception:
            pass
        pygame.quit()
        print("Cleanup completed.")

def spawn_vehicle(world, blueprint_library):
    """Spawn vehicle at random location"""
    try:
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        spawn_points = world.get_map().get_spawn_points()
        if spawn_points:
            spawn_point = random.choice(spawn_points)
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            print(f"Spawned vehicle: {vehicle.type_id}")
            return vehicle
        return None
    except Exception as e:
        print(f"Error spawning vehicle: {e}")
        return None

def main():
    print("CARLA Vehicle Control & Navigation - Phase 3")

    vehicle = None
    sensor_manager = None

    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        world = client.load_world('Town01')

        # Set up pygame
        pygame.init()
        display = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("CARLA Vehicle Control - Waypoint Navigation")

        # Spawn vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle = spawn_vehicle(world, blueprint_library)

        if vehicle:
            # Initialize controllers
            sensor_manager = SensorManager(world, vehicle)
            # vehicle_controller = VehicleController(vehicle, world)

            # Add sensors
            sensor_manager.setup_sensors(blueprint_library, display)
            sensor_manager.add_lidar(blueprint_library)
            sensor_manager.add_collision_sensor(blueprint_library)

            # Start autonomous vehicle system
            auto_vehicle_system = AutonomousVehicleSystem(vehicle, world, sensor_manager)
            auto_vehicle_system.display = display
            auto_vehicle_system.run()
    except Exception as e:
        print(f"Error in main: {e}")

if __name__ == "__main__":
    main()
