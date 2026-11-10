import cv2
import numpy as np
import time

# using pydobot instead of DobotEDU to run the script without Magicbox
# DobotEDU and Magicbox are using an older version of Python which can causes certain library issues
import pydobot
from pydobot.message import Message

# make sure to change /dev/ttyACM0 to the USB port connecting to Magician Lite
print("Connecting to Magician Lite...")
device = pydobot.Dobot(port='/dev/ttyACM0', verbose=False)
print("Connected.\n")

# make sure to change CAMERA_INDEX to match the ID of the robot's camera
CAMERA_INDEX = 2  
blue_calibrated = []
yellow_calibrated = []

# hardcoded Z values for picking/hovering and the robot's home position (matching the home position in DobotLab)
z_pick = -15
z_hover = 120    
home_pos = [240, 0, 145, 0]


# scan the workspace and add the pixel coordinates of blocks with the same colour into separate lists
# using OpenCV to detect colour and create a black white mask to increase accuracy when calculating the centre point
def scan_blocks():
    blue_list = []
    yellow_list = []
    red_list = []
    
    cap = cv2.VideoCapture(CAMERA_INDEX) 
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return blue_list, yellow_list
    
    for _ in range(10):
        ret, frame = cap.read()
        time.sleep(0.1)

    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab a frame.")
        cap.release()
        return blue_list, yellow_list

    print("Taking snapshot and scanning for blocks...")
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in blue_contours:
        area = cv2.contourArea(contour)
        if area > 500:  
            x, y, w, h = cv2.boundingRect(contour)
            centre_x = int(x + (w / 2))
            centre_y = int(y + (h / 2))
            blue_list.append((centre_x, centre_y))
            
    lower_yellow = np.array([20, 100, 100]) 
    upper_yellow = np.array([40, 255, 255])
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    
    yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in yellow_contours:
        area = cv2.contourArea(contour)
        if area > 500:  
            x, y, w, h = cv2.boundingRect(contour)
            centre_x = int(x + (w / 2))
            centre_y = int(y + (h / 2))
            
            yellow_list.append((centre_x, centre_y))
            
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask_red1, mask_red2)
    
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > 500:  
            x, y, w, h = cv2.boundingRect(contour)
            centre_x = int(x + (w / 2))
            centre_y = int(y + (h / 2))
            
            red_list.append((centre_x, centre_y))
        
    cap.release()
    
    return blue_list, yellow_list, red_list


# pass the list of the pixel coordinates of the colour for calibration and get the transformation matrix
# we are using the red block in the demonstration, but it can be any from red/blue/yellow which is placed on point A/C/D/F on the mat
def calibration(list):
    
    if len(list) != 4:
        print(f"Calibration Error: Expected 4 red blocks, but found {len(list)}.")
        return None
    
    print("Calibrating camera coordinates using the red blocks...")
    
    # sort the block to follow the same order bellow
    sorted_blocks = sorted(list, key=lambda pt: (pt[1], pt[0]), reverse=True)  
    camera_pixels = np.float32(sorted_blocks)

    # the fixed position in the robot's coordinate system of point A/C/D/F
    robot_mms = np.float32([
                       # facing the robot:
        [240, -100],   # point 1: Robot coords for top left
        [240, 100],    # point 2: Robot coords for top right
        [340, -100],   # point 3: Robot coords for bottom left
        [340, 100]     # point 4: Robot coords for bottom right
    ])

    # transformation matrix to convert the pixel coordinates to the robot's coordinate system
    transformation_matrix = cv2.getPerspectiveTransform(camera_pixels, robot_mms)
    
    print("Calibration finished.")
    
    return transformation_matrix


# helper function that passes the pixel coorinates and the matrix to get the real X and Y values
def pixel_to_robot(pixel_x, pixel_y, matrix):
    pts = np.float32([[[pixel_x, pixel_y]]])
    robot_coords = cv2.perspectiveTransform(pts, matrix)
    real_x = float(robot_coords[0][0][0])
    real_y = float(robot_coords[0][0][1])
    return real_x, real_y


# pick and place process
def pap(list):

    # Z value for stacking is 10 mm higher than before in every loop
    # starting from the same as z_pick
    z_stack = z_pick + 10
    place = list[0]
    device.move_to(home_pos[0], home_pos[1], home_pos[2], home_pos[3], wait=True)
    for block in list[1:]:
        device.move_to(block[0], block[1], z_hover, home_pos[3], wait=True)
        device.grip(False)
        time.sleep(1)
        device.move_to(block[0], block[1], z_pick, home_pos[3], wait=True)
        device.grip(True)
        time.sleep(1)
        device.move_to(block[0], block[1], z_hover, home_pos[3], wait=True)
        device.move_to(place[0], place[1], z_hover, home_pos[3], wait=True)
        device.move_to(place[0], place[1], z_stack, home_pos[3], wait=True)
        device.grip(False)
        time.sleep(1)
        device.move_to(place[0], place[1], z_hover, home_pos[3], wait=True)
        z_stack += 10
    device.move_to(home_pos[0], home_pos[1], home_pos[2], home_pos[3], wait=True)
    

# sending a message to the robot to shut down the pneumatic gripper
def shutdown():
    msg = Message()
    msg.id = 63
    msg.ctrl = 0x03
    msg.params = bytearray([0, 0]) 
    device._send_command(msg)
    print("Bye bye.")


if __name__ == "__main__":
    blocks = scan_blocks()
    matrix = calibration(blocks[2])
    
    for block in blocks[0]:
        blue_calibrated.append(pixel_to_robot(block[0], block[1], matrix))
    for block in blocks[1]:
        yellow_calibrated.append(pixel_to_robot(block[0], block[1], matrix))
    
    print("Sorting blue blocks...")
    print(blue_calibrated)
    pap(blue_calibrated)
    print("Sorting yellow blocks...")
    print(yellow_calibrated)
    pap(yellow_calibrated)
    
    print("Finished. Shutting down...") 
    shutdown()