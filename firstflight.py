from codrone_edu.drone import *

drone = Drone()
drone.pair()
drone.set_drone_LED(255,255,255,255)
drone.drone_buzzer(Note.C4, 1000)
drone.drone_buzzer(Note.D4, 1000)
drone.drone_buzzer(Note.E4, 1000)

drone.close()