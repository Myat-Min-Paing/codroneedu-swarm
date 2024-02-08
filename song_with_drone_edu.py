from serial.tools import list_ports
from codrone_edu.drone import *
from threading import Thread

#Want to fly porcess
def droneHandler(drone, portname):
    try:
        drone.pair(portname)
        drone.set_drone_LED(255,255,255,255)
        drone.drone_buzzer(Note.C4, 500)
        drone.drone_buzzer(Note.D4, 500)
        drone.drone_buzzer(Note.E4, 500)
        drone.drone_buzzer(Note.C4, 500)
        drone.set_drone_LED(255,255,0,255)
        
        drone.drone_buzzer(Note.C4, 500)
        drone.drone_buzzer(Note.D4, 500)
        drone.drone_buzzer(Note.E4, 500)
        drone.drone_buzzer(Note.C4, 500)
        drone.set_drone_LED(255,0,255,255)
        
        drone.drone_buzzer(Note.E4, 500)
        drone.drone_buzzer(Note.F4, 500)
        drone.drone_buzzer(Note.G4, 1000)
        drone.set_drone_LED(0,255,255,255)
        
        drone.drone_buzzer(Note.E4, 500)
        drone.drone_buzzer(Note.F4, 500)
        drone.drone_buzzer(Note.G4, 1000)
        drone.set_drone_LED(127,255,255,255)
        
        drone.drone_buzzer(Note.G4, 250)
        drone.drone_buzzer(Note.A4, 250)
        drone.drone_buzzer(Note.G4, 250)
        drone.drone_buzzer(Note.F4, 250)
        drone.drone_buzzer(Note.E4, 500)
        drone.drone_buzzer(Note.C4, 500)
        drone.set_drone_LED(255,127,255,255)
        
        drone.drone_buzzer(Note.G4, 250)
        drone.drone_buzzer(Note.A4, 250)
        drone.drone_buzzer(Note.G4, 250)
        drone.drone_buzzer(Note.F4, 250)
        drone.drone_buzzer(Note.E4, 500)
        drone.drone_buzzer(Note.C4, 500)
        drone.set_drone_LED(255,255,127,255)
        
        drone.drone_buzzer(Note.D4, 500)
        drone.drone_buzzer(Note.G3, 500)
        drone.drone_buzzer(Note.C4, 1000)
        drone.set_drone_LED(127,0,127,255)
        
        drone.drone_buzzer(Note.D4, 500)
        drone.drone_buzzer(Note.G3, 500)
        drone.drone_buzzer(Note.C4, 1000)
        drone.set_drone_LED(127,127,127,255)
        drone.takeoff()
        drone.set_drone_LED(0,255,255,255)
        drone.set_drone_LED(255,0,255,255)
        drone.set_drone_LED(255,255,0,255)
        drone.hover(3)
        drone.land()
        drone.close()
    except KeyboardInterrupt:
        drone.land()
        drone.close
    except Exception:
        drone.land()
        drone.close()
    
def scanDrone():
    x = list(list_ports.comports(include_links=True)) # detect all ports
    print(f"x value is {x}")
    portnames = []
    for element in x:
        if element.vid == 1155: # only find ports associated with controller ID
            portname = element.device
            print("Detected: ", portname)
            portnames.append(str(portname))
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


    