from serial.tools import list_ports
from codrone_edu.drone import *
from threading import Thread

# Function to handle drone movements
def droneHandler(drone, portname, other_drones, delay):
    try:
        print(f"Connecting to drone on port: {portname}")
        drone.pair(portname)

        # Synchronize takeoff for other drones
        for other_drone in other_drones:
            other_drone.pair()

        # Perform coordinated up-down crossing pattern movements
        if drone in other_drones:  # Middle drone goes down
            drone.set_drone_LED(255, 140, 0, 255)  # Move down
        else:  # Side drones go up
            drone.set_drone_LED(255, 255, 255, 255)  # Move up

        drone.land()
        for other_drone in other_drones:
            other_drone.land()  # Land other drones simultaneously
    except Exception as e:
        print(f"Error in droneHandler: {e}")
    finally:
        drone.close()

# Scan available drone ports
def scanDrone():
    x = list(list_ports.comports(include_links=True))
    print(f"Available ports: {x}")
    portnames = [element.device for element in x if element.vid == 1155]
    print(f"Detected drone ports: {portnames}")
    return portnames

# Create a list of Drone instances
drones = [Drone() for _ in range(3)]  # Assuming you have 3 drones

# Create and start threads for each drone
threads = []
delay_between_drones = 5  # seconds

try:
    for drone, portname in zip(drones, scanDrone()):
        other_drones = [d for d in drones if d != drone]  # Other drones in the group
        t = Thread(target=droneHandler, args=(drone, portname, other_drones, delay_between_drones))
        t.daemon = True
        t.start()
        threads.append(t)

    # Wait for all threads to finish before exiting the program
    for t in threads:
        t.join()

finally:
    # Close all drone connections
    for drone in drones:
        drone.close()
