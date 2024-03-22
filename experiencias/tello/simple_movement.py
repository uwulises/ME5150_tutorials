from djitellopy import Tello
import time

tello = Tello()
tello.connect()

print(tello.get_battery())
tello.streamon()
tello.takeoff()
tello.send_control_command("go -30 0 10 10")
print(tello.get_height())
tello.send_control_command("go 30 0 10 10")
print(tello.get_height())

tello.streamoff()
tello.land()