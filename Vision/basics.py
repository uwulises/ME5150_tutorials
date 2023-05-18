import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image
image = cv2.imread("Vision/taylor.jpg")
class ImageProcessing:
    def __init__(self, image):
        self.img = image
        self.gray_img = None
        self.binary_image = None
        self.rgb_image = None
        self.scaled_image = None
        self.rotated_image = None
        self.equalized_image = None
        self.dilated_image = None
        self.eroded_image = None

    def showImg(self, image, label_img = 'Image'):
        cv2.imshow(label_img, image)
    
    def saveImage(self, image, path_img = 'image.png'):
        cv2.imwrite(path_img, image)
    
    def displayColorSpaces(self):
        self.showImg(self.img, 'Original Image')
        self.showImg(self.rgb_image, 'RGB Image')
        self.showImg(self.gray_image, 'Grayscale Image')
        self.showImg(self.binary_image, 'Binary Image')

        # Wait for a key press and then close the windows
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def toGray(self):
        self.gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, self.binary_image = cv2.threshold(self.gray_image, 128, 255, cv2.THRESH_BINARY)
        return self.binary_image

    def toRGB(self):
        self.rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return self.rgb_image

    def toGrayScale(self):
        self.gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return self.gray_image



    #--------------------------------- Preprocesamiento ------------------

    def scaling(self, width, height):
        # Resize the image
        self.scaled_image = cv2.resize(self.img, (width, height))
        return self.scaled_image

    def rotation(self, ang = 45):
        # Get the image dimensions
        height, width = self.img.shape[:2]

        # Calculate the rotation matrix
        rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), ang, 1)

        # Perform the rotation
        self.rotated_image = cv2.warpAffine(self.img, rotation_matrix, (width, height))
        return self.rotated_image

    def equalizate(self):
        self.equalized_image = cv2.equalizeHist(self.gray_image)
        return self.equalized_image

    def dilate(self, k = 5):
        # Define the kernel size and shape for dilatation
        kernel = np.ones((k, k), np.uint8)

        # Perform dilatation on the image
        self.dilated_image = cv2.dilate(self.img, kernel, iterations=1)
        return self.dilated_image

    def erosion(self, k = 5):
        # Define the kernel size and shape for erosion
        kernel = np.ones((k, k), np.uint8)

        # Perform erosion on the image
        self.eroded_image = cv2.erode(self.img, kernel, iterations=1)
        return self.eroded_image

image_proc = ImageProcessing(image)
dilate = image_proc.dilate()
image_proc(dilate, 'Dilated Image')