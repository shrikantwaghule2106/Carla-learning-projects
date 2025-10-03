

import carla
import random
import time
import os

def monitor_performance():
    """Monitor system performance"""
    try:
        import psutil
        import GPUtil

        cpu_percent = psutil.cpu_percent(interval=1)
        ram = psutil.virtual_memory()
        ram_percent = ram.percent

        gpus = GPUtil.getGPUs()
        if gpus:
            gpu = gpus[0]
            print(f"🖥️  Performance: CPU: {cpu_percent}% | RAM: {ram_percent}% | GPU: {gpu.load*100:.1f}% | VRAM: {gpu.memoryUsed}MB/{gpu.memoryTotal}MB")
        else:
            print(f"🖥️  Performance: CPU: {cpu_percent}% | RAM: {ram_percent}%")
    except Exception as e:
        print(f"⚠️  Performance monitoring unavailable: {e}")

def main():
    print("="*60)
    print("CARLA Project 1: Vehicle Spawn and Autopilot")
    print(f"Working from: {os.getcwd()}")
    print("="*60)

    # Connect to CARLA
    print("\n[1/5] Connecting to CARLA server...")
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        print("✅ Connected successfully!")
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        print("\nTroubleshooting:")
        print("1. Run: D:\\Softwares\\CARLA\\run_carla_low.bat")
        print("2. Wait 1-2 minutes for CARLA to fully start")
        print("3. Try again")
        return

    # Get world
    world = client.get_world()
    current_map = world.get_map().name
    print(f"✅ Current map: {current_map}")

    # Set synchronous mode
    print("\n[2/5] Configuring synchronous mode...")
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # 20 FPS
    world.apply_settings(settings)
    print("✅ Synchronous mode enabled (20 FPS)")

    # Get blueprint library
    blueprint_library = world.get_blueprint_library()

    # Choose vehicle
    print("\n[3/5] Spawning vehicle...")
    vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    # Get spawn point
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("❌ No spawn points available!")
        return

    spawn_point = random.choice(spawn_points)

    # Spawn vehicle
    try:
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"✅ Spawned: {vehicle.type_id}")
        print(f"   Location: X={spawn_point.location.x:.1f}, Y={spawn_point.location.y:.1f}")
    except Exception as e:
        print(f"❌ Spawn failed: {e}")
        settings.synchronous_mode = False
        world.apply_settings(settings)
        return

    # Attach spectator camera
    spectator = world.get_spectator()

    try:
        # Enable autopilot
        print("\n[4/5] Enabling autopilot...")
        vehicle.set_autopilot(True)
        print("✅ Autopilot enabled")

        # Monitor vehicle
        print("\n[5/5] Monitoring vehicle (30 seconds)...")
        print("Press Ctrl+C to stop early\n")

        for i in range(30):
            world.tick()  # Update simulation

            # Update spectator to follow vehicle (bird's eye view)
            transform = vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location + carla.Location(z=50),
                    carla.Rotation(pitch=-90)
                )
            )

            # Print status every 5 seconds
            if i % 5 == 0:
                velocity = vehicle.get_velocity()
                speed_kmh = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
                location = vehicle.get_location()

                print(f"\n⏱️  Time: {i}s")
                print(f"   Speed: {speed_kmh:.1f} km/h")
                print(f"   Position: X={location.x:.1f}, Y={location.y:.1f}")
                monitor_performance()

            time.sleep(1)

        # Final report
        print("\n" + "="*60)
        print("✅ Test completed successfully!")
        final_location = vehicle.get_location()
        final_velocity = vehicle.get_velocity()
        final_speed = 3.6 * (final_velocity.x**2 + final_velocity.y**2 + final_velocity.z**2)**0.5

        print(f"\n📊 Final Statistics:")
        print(f"   Duration: 30 seconds")
        print(f"   Final Speed: {final_speed:.1f} km/h")
        print(f"   Final Position: X={final_location.x:.1f}, Y={final_location.y:.1f}")

        distance = ((final_location.x - spawn_point.location.x)**2 +
                   (final_location.y - spawn_point.location.y)**2)**0.5
        print(f"   Distance Traveled: {distance:.1f} meters")
        print("="*60)

    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")

    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")

        # Restore async mode
        settings.synchronous_mode = False
        world.apply_settings(settings)

        # Destroy vehicle
        if vehicle.is_alive:
            vehicle.destroy()
            print("✅ Vehicle destroyed")

        print("✅ Cleanup complete!")

if __name__ == '__main__':
    try:
        # Check and install missing packages
        try:
            import GPUtil
            import psutil
        except ImportError:
            print("Installing performance monitoring packages...")
            import subprocess
            subprocess.check_call(['pip', 'install', 'gputil', 'psutil'])
            print("✅ Packages installed! Please run the script again.")
            exit(0)

        main()

    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
