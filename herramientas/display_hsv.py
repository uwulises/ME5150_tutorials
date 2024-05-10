import cv2
import numpy as np

# Callback function for slider changes
def on_slider_change(value):
    # Retrieve current slider values
    h1 = cv2.getTrackbarPos('H1', 'Sliders Window')
    s1 = cv2.getTrackbarPos('S1', 'Sliders Window')
    v1 = cv2.getTrackbarPos('V1', 'Sliders Window')
    h2 = cv2.getTrackbarPos('H2', 'Sliders Window')
    s2 = cv2.getTrackbarPos('S2', 'Sliders Window')
    v2 = cv2.getTrackbarPos('V2', 'Sliders Window')

# Create a window to display the sliders
cv2.namedWindow('Sliders Window')

# Create three sliders
cv2.createTrackbar('H1', 'Sliders Window', 0, 179, on_slider_change)
cv2.createTrackbar('S1', 'Sliders Window', 0, 255, on_slider_change)
cv2.createTrackbar('V1', 'Sliders Window', 0, 255, on_slider_change)
cv2.createTrackbar('H2', 'Sliders Window', 0, 179, on_slider_change)
cv2.createTrackbar('S2', 'Sliders Window', 0, 255, on_slider_change)
cv2.createTrackbar('V2', 'Sliders Window', 0, 255, on_slider_change)


""" FUNCIÓN DE PROCESAMIENTO DE IMÁGENES
Parametros:
    img: imagen a procesar
    color_hsv1: color en HSV
    color_hsv2: color en HSV
Retorno:
    brg: imagen en BGR
"""
def process_frame(img, color_hsv1, color_hsv2):
    # Convert the image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a mask using the color range
    mask_blue = cv2.inRange(hsv, color_hsv1, color_hsv2)

    # Convert the mask to a BGR image
    brg = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
    return brg



def main (path):
    image1 = np.zeros((240, 320, 3), dtype=np.uint8)
    image2 = np.zeros((240, 320, 3), dtype=np.uint8)
    proc_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    cap = cv2.VideoCapture(path)
    ret = 0
    
    while True:
        # Get the current slider values
        h1 = cv2.getTrackbarPos('H1', 'Sliders Window')
        s1 = cv2.getTrackbarPos('S1', 'Sliders Window')
        v1 = cv2.getTrackbarPos('V1', 'Sliders Window')
        h2 = cv2.getTrackbarPos('H2', 'Sliders Window')
        s2 = cv2.getTrackbarPos('S2', 'Sliders Window')
        v2 = cv2.getTrackbarPos('V2', 'Sliders Window')

        # Set the color in HSV
        color_hsv1 = np.array([[[h1, s1, v1]]], dtype=np.uint8)
        color_hsv2 = np.array([[[h2, s2, v2]]], dtype=np.uint8)

        color_bgr1 = cv2.cvtColor(color_hsv1, cv2.COLOR_HSV2BGR)
        color_bgr1 = tuple(map(int, color_bgr1[0, 0]))

        color_bgr2 = cv2.cvtColor(color_hsv2, cv2.COLOR_HSV2BGR)
        color_bgr2 = tuple(map(int, color_bgr2[0, 0]))

        image1[:] = color_bgr1
        image2[:] = color_bgr2

        # Display the image in the window
        hor1 = cv2.hconcat([image1, image2]) 

        # Read a frame from the webcam or video
        ret, frame = cap.read()

        # Check if frame reading was successful
        if ret:
            # Process the frame
            frame = cv2.resize(frame, (320, 240))
            proc_frame = process_frame(frame, color_hsv1[0][0], color_hsv2[0][0])
            hor2 = cv2.hconcat([frame, proc_frame])
            union = cv2.vconcat([hor1, hor2]) 
        else:
            if isinstance(path, int):
                # If webcam, break
                break
            else:
                # If video, restart
                cap = cv2.VideoCapture(path)
                continue
        
        cv2.imshow('Sliders Window', union)
        # Wait for the 'Esc' key to be pressed
        if cv2.waitKey(1) == 27:
            break

    # Close the window and release resources
    cv2.destroyAllWindows()
    cap.release()

# Run the main function
if __name__ == '__main__':

    """
    Cambiar path según video o bien, poner 0 para webcam (como entero)
    """

    # path = './multimedia/destacador.jpg'
    # path = './multimedia/pitufos.mp4'
    path = 0
    main(path)