from serial.tools import list_ports
from codrone_edu.drone import *

drone_objects = []
num_drones = 0

#Want to fly porcess
def droneFlightProc(drone):
    drone.takeoff()
    drone.hover(2)
    drone.land()

x = list(list_ports.comports(include_links=True)) # detect all ports
print(f"x value is {x}")
portnames = []
for element in x:
    if element.vid == 1155: # only find ports associated with controller ID
        portname = element.device
        print("Detected: ", portname)
        portnames.append(str(portname))

drone1 = Drone() # create drone objects
drone2 = Drone()

drone1.pair(portnames[0]) # first index is 0
print("Paired drone 1 at port\n", portnames[0])
drone2.pair(portnames[1]) # second index is 1
print("Paired drone 2 at port\n", portnames[1])

#drone1.set_drone_LED(255,0,255,255) # drone 1 purple
#drone2.set_drone_LED(0,255,0,255) # drone 2 green

drone1.takeoff()
drone2.takeoff()
drone1.hover(2)
drone2.hover(2)
drone1.land()
drone2.land()

#calling function
#droneFlightProc(drone1)
#droneFlightProc(drone2)

drone1.close() # don't forget to close!
drone2.close()


    