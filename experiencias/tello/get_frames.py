from djitellopy import tello
import cv2

drone = tello.Tello()
drone.connect()
drone.streamon()
get_frames = drone.get_frame_read(with_queue = False, max_queue_len=64)

while True:
    img = get_frames.frame
    cv2.imshow('stream', img)
    cv2.waitKey(1)

drone.streamoff()