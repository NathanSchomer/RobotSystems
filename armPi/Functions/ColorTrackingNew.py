#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from ImageUtils import ImageUtils
from MoveUtils import MoveUtils

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red',)
# set detection color
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color
    return (True, ())


# initial position
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

#Set the RGB light color of the expansion board to make it consistent with the color to be tracked
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

count = 0
track = False
_stop = False
get_roi = False
center_list = []
first_move = True
__isRunning = False
detect_color = 'None'
action_finish = True
start_pick_up = False
start_count_t1 = True
# Variable reset
def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1
    
    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True

# app initialization call
def init():
    print("ColorTracking Init")
    move_utils = MoveUtils()
    move_utils.initMove()

# App start playing method call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# app stop gameplay call
def stop():
    global _stop 
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")

# App exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0
# Robotic arm moves thread
def move():
    global rect
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global center_list, count
    global start_pick_up, first_move

    # Place coordinates of different colors (x, y, z)
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }

    move_utils = MoveUtils()

    while True:
        if __isRunning:
            if first_move and start_pick_up: # When an object is detected for the first time
                action_finish = False
                set_rgb(detect_color)
                setBuzzer(0.1)               
                unreachable = move_utils.first_move(world_X, world_Y)
                start_pick_up = False
                first_move = False
                action_finish = True

            elif not first_move and not unreachable: # If it is the tracking stage
                set_rgb(detect_color)
                if track: # If it is the tracking stage
                    if not __isRunning: # Stop and exit flag detection
                        continue

                    move_utils.arm_raise(world_X, world_Y)
                    track = False

                if start_pick_up: #If the object hasn't moved for a while???Start gripping
                    
                    action_finish = False

                    if not __isRunning: # Stop and exit flag detection
                        continue

                    move_utils.gripper_open()
                    angle = getAngle(world_X, world_Y, rotation_angle)
                    move_utils.gripper_angle(angle)
                    move_utils.arm_lower(world_X, world_Y)
                    
                    if not __isRunning:
                        continue

                    move_utils.gripper_close()
                    move_utils.arm_raise(world_X, world_Y)
                    
                    if not __isRunning:
                        continue

                    # Sort and place different colored squares
                    world_X = coordinate[detect_color][0]
                    world_Y = coordinate[detect_color][1]  
                    move_utils.move_arm(world_X, world_Y)
                    
                    if not __isRunning:
                        continue

                    angle = getAngle(coordinate[detect_color][0],
                                     coordinate[detect_color][1], -90)
                    move_utils.gripper_angle(angle)

                    if not __isRunning:
                        continue

                    X, Y, Z = coordinate[detect_color]
                    move_utils.arm_lower_gentle(X, Y, Z)
                    move_utils.gripper_open()
                    
                    if not __isRunning:
                        continue                    

                    move_utils.arm_raise(world_X, world_Y)
                    move_utils.initMove()

                    detect_color = 'None'
                    first_move = True
                    get_roi = False
                    action_finish = True
                    start_pick_up = False
                    set_rgb(detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)

# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
last_x, last_y = 0, 0
def run(img):
    global roi
    global rect
    global count
    global track
    global get_roi
    global center_list
    global __isRunning
    global unreachable
    global detect_color
    global action_finish
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1
    global start_pick_up, first_move

    img_utils = ImageUtils()
   
    img_copy = img_utils.draw_plus(img.copy())
    
    if not __isRunning:
        return img
    
    frame_lab = img_utils.preprocess(img_copy, get_roi, roi, start_pick_up)
    
    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:   # if we haven't started picking stuff up yet
        for i in color_range:   # for each color in possible colors
            if i in __target_color: 
                detect_color = i 

                ######################### Detect a specific color ######################################
                # in:  frame_lab, color_range
                # out: areaMaxContour, area_max
                areaMaxContour, area_max = img_utils.detect_color_contour(frame_lab, color_range[detect_color])
                ########################################################################################

        # only use areas of sufficient size (determined by this magic number)
        if area_max > 2500:  # Have found the largest area

            ############## Contour to Box ################3
            # in:  areaMaxContour
            # out: box
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            ########################################
            
            get_roi = True
            ############ Box to world coordinates ################
            # in:  box
            # out: roi
            roi = getROI(box) #Get roi area

            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # Get the center coordinates of the block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size) #Convert to real world coordinates
            ####################################################
           
            img = img_utils.label_img(img, box, world_x, world_y, range_rgb[detect_color])
            
            # Compare to last coordinate to determine if we have to move
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))
            last_x, last_y = world_x, world_y
            track = True
            #print(count,distance)

            # Cumulative judgment
            if action_finish:
                if distance < 0.3:
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        count = 0
                        center_list = []
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []
    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', )
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
