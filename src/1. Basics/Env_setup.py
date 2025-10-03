import carla
import random

def main():
    print("CARLA Environment Setup")

    # Connect to the client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world
    world = client.get_world()
    print(f"Connected to world: {world}")

    # Display available maps
    available_maps = client.get_available_maps()
    print(f"Available maps: {len(available_maps)}")

    # Cleanup function
    def cleanup():
        print("Cleanup completed")

    return cleanup

if __name__ == "__main__":
    main()
