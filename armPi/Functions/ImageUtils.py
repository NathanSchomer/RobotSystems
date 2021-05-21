import cv2
import sys
sys.path.append('/home/pi/ArmPi/')
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import numpy as np


class ImageUtils():

    def __init__(self):
        return

    # Find the contour with the largest area
    # The parameter is a list of contours to be compared
    @staticmethod
    def getAreaMaxContour(contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Traverse all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only when the area is greater than 300，The contour with the largest area is effective，To filter out interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour

    def draw_plus(self, img):
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        return img

    def preprocess(self, img, get_roi, roi, start_pick_up):

        size = (640, 480)

        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        #If a recognized object is detected in a certain area，Then keep detecting the area until there is no
        if get_roi and start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, roi, size)    

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        return frame_lab

    def detect_color_contour(self, frame_lab, color_range):

        frame_mask = cv2.inRange(frame_lab, color_range[0], color_range[1])  # Perform bit operations on the original image and mask

        # isolate detecteced "blobs" then find maximum region
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Closed operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find the outline
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # Find the largest contour

        return areaMaxContour, area_max

    def label_img(self, img, box, world_x, world_y, color):
        cv2.drawContours(img, [box], -1, color, 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1) #Draw center point
        return img
