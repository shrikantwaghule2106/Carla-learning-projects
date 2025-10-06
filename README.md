# CARLA Learning Projects

Learning Basics

# Points to consider -

1. Run the CARLA setup before executing the code.
2. Ensure CARLA setup is running on localhost 2000
   netstat -ano | findstr :2000

## 🚗 Project Structure

### Phase 1: Fundamentals

- Setting up Environment setup and understanding basic navigation, vehicle spawning anf camera visualization.

- Libraries used -

  1. CARLA - For simulation
  2. PyGame - For Display
  3. Numpy for image processing
  4. random - For random selection of all parameters

- Code structure -

  1. Spawn vehicle - Define a function, get a random vehicle blueprint, fetch all possible spawn points.Choose one random spawn point from the set if they exist, else return none.
  2. Main -
     a. Initialize variables
     b. Connect to the client with timeout of 10 secs.
     c. Fetch the current world and print data regarding it.
     d. Get and display current world climate conditions.
     e. Enlist all availlable maps
     f. Spawn a random variable
     g. Setup Pygame window as needed for camera display.
     h. If vehicle spawned then add a RGB camera to it.

     i.Process Camera images - 1. Convert raw data to numpy array. Data is send in form of buffer of raw bytes as 8 Bit integers (0-255) 2. Reshape - Convert flat 1d array to 2d image grid. CARLA uses BGRA (Alpha) format hence 4 channels. 3. Omit alpha as we don't need it. 4. Reverse the order BGR to RGB by using -1. 5. convert numpy array (h,w) to pygame array (w,h) by swapping. 6. Display image to main display. Use Blit(0,0) to map image to position of top left corner. 7. Flip to display
     j. Start camera sensor and register the callback function..
     k. Move the vehicle throttle=0.3 is 30% throttle.
     l. Run loop for 10 secs.
     m. Cleanup all the settings.

### Phase 2: Sensor Systems

- RGB camera, LIDAR, and collision sensors
- Data processing and sensor fusion

### Phase 3: Vehicle Control

- Manual and autonomous control systems
- Waypoint navigation and PID controllers

### Phase 4: Traffic Simulation

- Traffic management and pedestrian behavior
- Scenario design and execution

### Phase 5: Advanced Applications

- Object detection and avoidance
- Custom AI behaviors

### Phase 6: Data & Analysis

- Data collection and recording
- Simulation replay and analysis

## 🛠️ Setup

```bash
pip install -r requirements.txt
```
