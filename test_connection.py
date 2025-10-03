"""
CARLA Connection Test
Quick script to verify your installation works
Run this AFTER starting CarlaUE4.exe
"""

import sys

print("="*50)
print("CARLA Connection Test")
print("="*50)

# Test 1: Check if carla module is installed
print("\n[1/4] Checking CARLA module...")
try:
    import carla
    print("CARLA module found!")
   # print(f"   Version: {carla.__version__ if hasattr(carla, '__version__') else 'Unknown'}")
except ImportError as e:
    print("CARLA module not found!")
    print("   Run: pip install carla==0.9.13")
    sys.exit(1)

# Test 2: Check other dependencies
print("\n[2/4] Checking dependencies...")
try:
    import pygame
    import numpy
    import cv2
    print("All dependencies installed!")
except ImportError as e:
    print(f"Missing: {e.name}")
    print("   Run: pip install pygame numpy opencv-python")
    sys.exit(1)

# Test 3: Connect to CARLA server
print("\n[3/4] Connecting to CARLA server...")
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    print("Connected to CARLA!")
except Exception as e:
    print("Connection failed!")
    print(f"   Error: {e}")
    print("\n   Troubleshooting:")
    print("   1. Is CarlaUE4.exe running?")
    print("   2. Check if port 2000 is open")
    print("   3. Try restarting CARLA")
    sys.exit(1)

# Test 4: Get world info
print("\n[4/4] Getting world information...")
try:
    world = client.get_world()
    map_name = world.get_map().name
    actors = world.get_actors()

    print("World loaded successfully!")
    print(f"   Current Map: {map_name}")
    print(f"   Active Actors: {len(actors)}")

    # Get available maps
    available_maps = client.get_available_maps()
    print(f"\n📍 Available maps ({len(available_maps)}):")
    for map_path in sorted(available_maps)[:5]:  # Show first 5
        map_name = map_path.split('/')[-1]
        print(f"   - {map_name}")
    if len(available_maps) > 5:
        print(f"   ... and {len(available_maps) - 5} more")

except Exception as e:
    print(f"Error getting world info: {e}")
    sys.exit(1)

# Success!
print("\n" + "="*50)
print("🎉 ALL TESTS PASSED!")
print("="*50)
print("\nYour CARLA installation is working correctly!")
print("You can now run the project scripts.")
print("\nRecommended next steps:")
print("1. Try spawning a vehicle (Project 1)")
print("2. Experiment with different maps")
print("3. Attach sensors to vehicles")
print("\nHappy coding! 🚗💨")
