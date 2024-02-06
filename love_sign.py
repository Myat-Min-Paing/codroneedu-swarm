from serial.tools import list_ports
from codrone_edu.drone import *
from threading import Thread

# Want to fly process
def droneHandler(drone, portname):
    try:
        print(f"Connecting to drone on port: {portname}")
        drone.pair(portname)
        drone.takeoff()
        new_drone = Drone()
        new_drone_port = scanDrone()[1]  # Replace with the appropriate index
        new_drone.pair(new_drone_port)
        new_drone.go(0, 0, 0, 20, 1)
        drone.close()
    except Exception as e:
        print(f"Error: {e}")
        drone.land()
        drone.close()
    except KeyboardInterrupt:
        drone.land()
        drone.close()

def scanDrone():
    x = list(list_ports.comports(include_links=True))  # detect all ports
    print(f"x value is {x}")
    portnames = [element.device for element in x if element.vid == 1155]
    print(f"Detected drone ports: {portnames}")
    return portnames

# Create a list of Drone instances
drones = [Drone() for _ in range(len(scanDrone()))]

# Create and start threads for each drone
threads = []
for drone, portname in zip(drones, scanDrone()):
    t = Thread(target=droneHandler, args=(drone, portname))
    t.daemon = True
    t.start()
    threads.append(t)

# Wait for all threads to finish before exiting the program
for t in threads:
    t.join()
