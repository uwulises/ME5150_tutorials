from djitellopy import Tello
import time
tello = Tello()

tello.connect()
print(tello.get_battery())
tello.streamon()
tello.takeoff()
tello.move_back(50)
tello.move_forward(50)
tello.send_control_command("forward 50")
tello.send_control_command("go 30 0 100 10")
print(tello.get_height())

tello.streamoff()
tello.land()