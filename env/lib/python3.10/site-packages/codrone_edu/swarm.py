from serial.tools import list_ports
from codrone_edu.drone import *
from time import sleep
from threading import Thread


class Swarm:
    def __init__(self):
        self.drone_objects = []
        self.num_drones = 0

    def auto_connect(self):

        x = list(list_ports.comports(include_links=True))
        portnames = []

        # Append all of the correct drone portnames in the portnames list
        for element in x:
            if element.vid == 1155:
                portname = element.device
                print("Detected: ", portname)
                portnames.append(str(portname))
        print("")

        self.num_drones = len(portnames)

        for i in range(self.num_drones):
            self.drone_objects.append(Drone())

        for i in range(self.num_drones):
            self.drone_objects[i].pair(portnames[i])
            print("Paired drone at port ", portnames[i])
            print("")

            #self.drone_objects[i].setEventHandler(DataType.State, self.event_state)

        for i in range(self.num_drones):
            pass

    def close_all(self):

        for drone in self.drone_objects:
            drone.close()

    def all_takeoff(self):
        for drone in self.drone_objects:
            Thread(target=drone.takeoff).start()
        sleep(4)

    def all_land(self):
        for drone in self.drone_objects:
            Thread(target=drone.land).start()
        sleep(4)

    def all_move(self, r, p, y, t, seconds):

        timeout = seconds
        init_time = time.time()

        while time.time() - init_time < timeout:
            for drone in self.drone_objects:
                drone.sendControl(r, p, y, t)
                sleep(0.05)

    def all_hover(self, seconds):

        timeout = seconds
        init_time = time.time()

        while time.time() - init_time < timeout:
            for drone in self.drone_objects:
                drone.sendControl(0, 0, 0, 0)
                sleep(0.05)

    def start_threading(self,  *args):
        for thread in args:
            thread.start()

    def all_turn_degree(self, degree):
        for drone in self.drone_objects:
            Thread(target=drone.turn_degree, args=[90]).start()

    def all_flip(self):
        for drone in self.drone_objects:
            Thread(target=drone.flip, args=["back"]).start()
            sleep(0.05)
