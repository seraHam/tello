

#import the necessary packages

import cv2
import numpy as np
import apriltag
import math
import time
import pygame
import sys

from djitellopy import Tello


# declare variables

# drone
SPEED = 50
FPS = 20
# screen drawings
centers = []
LINE_LENGTH = 5
CENTER_COLOR = (255, 0, 0)
CORNER_COLOR = (0, 255, 180)
# moving drone
DESIRED_DISTANCE = 250 #cm
YAW_FACTOR = 5
THROTTLE_FACTOR = 5
TAG_SIZE = 18 # cm
n = 0
m = 0
distances = []
ids = [0]    
pitchspeed = 0
rollspeed = 0
throttlespeed = 0
yawspeed = 0
# ids for the gates
id_max_1 = 31
id_min_1 = 24
id_max_2 = 23
id_min_2 = 16
id_max_3 = 15
id_min_3 = 8


# create functions
#
# initialize drone
def initialize_drone():

    #initialize
    drone = Tello()
    drone.connect()
    drone.streamoff()
    drone.streamon()
    #set velocity
    drone.set_speed(SPEED)
    drone.send_rc_control(0, 0, 0, 0)
    drone.streamon()

    return drone

# initialize screen: using pygame
def initialize_screen():

    pygame.init()
    screen = pygame.display.set_mode((960, 720))
    pygame.display.set_caption("AprilTag Gates")
    screen.fill((0, 0, 0))

    return screen

# initialize detector
def initialize_detector():

    detector = apriltag.Detector()

    return detector


# define some functions

def get_drone_frame(drone):

    frame_read = drone.get_frame_read()
    frame = frame_read.frame

    return frame

def get_tag_corners(detector, gray_image):

    results = detector.detect(gray_image)
    corners = []

    for r in results:
        corners.append(r.corners)

    return corners, results


# define the drawing functions: used for the user interface
# the drone would still work without them
def draw_corners(image, corner):

    # draw corners
    corner = (int(corner[0]), int(corner[1]))
    image = cv2.line(image,
                     (corner[0] - LINE_LENGTH, corner[1]),
                     (corner[0] + LINE_LENGTH, corner[1]),
                     (0, 0, 255),
                     3)
    image = cv2.line(image,
                     (corner[0], corner[1] - LINE_LENGTH),
                     (corner[0], corner[1] + LINE_LENGTH),
                     (0, 0, 255),
                     3)
    

    return image


def draw_center(image, radius, center):

    # draw a circle on the center of the apriltag
    center = (int(center[0]), int(center[1]))
    #pygame.draw.circle(screen, CENTER_COLOR, center, LINE_LENGTH)
    image = cv2.circle(image, center, int(radius), (0, 0, 255), 2)
    return image

def plot_text(image, text, center):

    # plot text on the screen
    font = pygame.font.SysFont("Arial", 20)
    #text = font.render(text, True, (255, 0, 0))
   
    return cv2.putText(image, str(text), (center[0] + 10, center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

# create a function to calculate the center of the gate: where the drone is
# going to go to. A mean between the centers position of the Apriltags
def calculate_centers_mean_gate(center, ID, id_max, id_min):

    if ID <= id_max and ID >= id_min:
        
        centers.append(center)
        if len(centers) > 5:
            mean_x = np.mean([center[0] for center in centers])
            mean_y = np.mean([center[1] for center in centers])            

            # Store the means in a variable named "mean"
            mean = (int(mean_x), int(mean_y))

            return mean

        else:
            return None
    else:
        
        return None


# create a function to calculate the distance to the gate: a mean of the
# distances to the Apriltags
def calculate_distances_mean_gate(distance, ID, id_max, id_min):
    
    if ID <= id_max and ID >= id_min:
        
        distances.append(distance)
        if len(distances) > 5:
            d_mean = np.mean([distance for distance in distances])

            return d_mean
        
        else:
            return None
    else:
        
        return None


# define a function to calculate the velocity of the drone, based on the
# position of the center of the gate
def set_velocity(mean, d_mean):

    yawspeed = (int(mean[0] - 480) / YAW_FACTOR)

    throttlespeed = (int(mean[1] - 390) / THROTTLE_FACTOR)*(-1)

    pitchspeed = int((int(d_mean) - int(DESIRED_DISTANCE)) / 1.5)
    
    # ----------------------------------------------------------------------- ⚠️  ⚠️  ⚠️   
    # check throttle
    # ----------------------------------------------------------------------- ⚠️  ⚠️  ⚠️ 
    return yawspeed, throttlespeed, pitchspeed


def search():
   
    # not used in this version
    # could be used if the starting conditions are more flexible

    global m
    global n

    if m % 2 == 0:

        if n % 2 == 0:
            drone.rotate_clockwise(10)
        else:
            drone.rotate_counter_clockwise(10)
        n += 1
    
        if n % 2 == 0:
            drone.move_up(20)
        else:
            drone.move_down(20)
    else:
        if n % 2 == 0:
            drone.rotate_counter_clockwise(10)
        else:
            drone.rotate_clockwise(10)
        
    m += 1

# define functions to controll the drone manually
#
# when a key is pressed, the drone will move in the direction of the key
def keydown(key):

    global pitchspeed, rollspeed, throttlespeed, yawspeed
    if key == pygame.K_UP:
        pitchspeed = SPEED
    elif key == pygame.K_DOWN:
        pitchspeed = -SPEED
    elif key == pygame.K_LEFT:
        rollspeed = -SPEED
    elif key == pygame.K_RIGHT:
        rollspeed = SPEED
    elif key == pygame.K_w:
        throttlespeed = SPEED
    elif key == pygame.K_s:
        throttlespeed = -SPEED
    elif key == pygame.K_a:
        yawspeed = -SPEED
    elif key == pygame.K_d:
        yawspeed = SPEED

# when a key is released, the drone will stop moving
def keyup(key): 

    global pitchspeed, rollspeed, throttlespeed, yawspeed
    if key == pygame.K_UP or key == pygame.K_DOWN:
        pitchspeed = 0
    elif key == pygame.K_LEFT or key == pygame.K_RIGHT:
        rollspeed = 0
    elif key == pygame.K_w or key == pygame.K_s:
        throttlespeed = 0
    elif key == pygame.K_a or key == pygame.K_d:
        yawspeed = 0
    
    elif key == pygame.K_t:
        send_rc_control = True
        drone.takeoff()
    elif key == pygame.K_l:
        drone.land()

# define a function to send the speed commands to the drone
def update():

    
    if send_rc_control:

        drone.send_rc_control(rollspeed, pitchspeed, throttlespeed, yawspeed)



# --------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------

if __name__ == '__main__':

    # initialize the drone
    drone = initialize_drone()
    # initialize the screen
    screen = initialize_screen()
    # initialize the detector
    detector = initialize_detector()
    
    # make sure that the drone is not moving
    send_rc_control = False
    drone.send_rc_control(0, 0, 0, 0)
    
    drone.send_rc_control(0, 0, 0, 0)
    
    # take off
    drone.takeoff()
    
    # move to the starting position: could be different for a different gate
    # distribution
    drone.move_up(150)

    drone.move_back(400)
    
    # define the time between frames
    pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    # loop
    while True:
        
        frame_read = drone.get_frame_read()


        # -------------------------------------- gate 1 ------------------------------------------
        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎

        should_stop_gate_1 = False

        while not should_stop_gate_1:
            
            frame = frame_read.frame

            image = frame

            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(gray_image)
            
            mean = None
            d_mean = None
            centers = [0]
            centers.clear()
            distances = [0]
            distances.clear()

            if not detections:
                print("Not detections")
                
                #search()

                #time.sleep(1)

            else:
                for detect in detections:
                    print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))

                    # Calculate distance
                    d = math.sqrt(((detect.corners[0][0]
                                    - detect.corners[2][0]) **
                                   2 + (detect.corners[0][1]
                                        - detect.corners[2][1]) ** 2)/2)
                    distance = (166*100 / d)
                    
                    radius = distance/20
                   
                    mean = calculate_centers_mean_gate(detect.center, detect.tag_id, id_max_1, id_min_1)
                        
                    d_mean = calculate_distances_mean_gate(distance, detect.tag_id, id_max_1, id_min_1)
                
                if mean is not None and d_mean is not None:
                   
                    print("means are not none")

                    image = plot_text(image, "Gate 1", (20, 20))
                    
                    image = draw_center(image, 10, mean)

                    image = plot_text(image, str(d_mean), (20, 50))
                    
                    yawspeed, throttlespeed, pitchspeed = set_velocity(mean, d_mean)
                    rollspeed = 0
                    
                    for event in pygame.event.get():

                        if event.type == pygame.QUIT:
                            should_stop_gate_1 = True

                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_ESCAPE:
                                should_stop_gate_1 = True
                                drone.land()
                            else:
                                keydown(event.key)

                        elif event.type == pygame.KEYUP:
                            keyup(event.key)


                    drone.send_rc_control(int(rollspeed), int(pitchspeed), int(throttlespeed), int(yawspeed))


                    pygame.display.update()


                    # check if the drone is close enough to the center of the apriltag
                    if d_mean < DESIRED_DISTANCE:
                        print("distance < desired distance")
                        
                        drone.send_rc_control(0, 0, 0, 0)
                       
                        should_stop_gate_1 = True
                    

                    # draw the center of the apriltag
                    for detect in detections:                
                        image = draw_center(image, 5, detect.center)
                        
                        # draw the corners of the apriltag
                
                        for corner in detect.corners:

                            image = draw_corners(image, corner)
            
            update()

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    should_stop = True

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                        drone.land()
                    else:
                        keydown(event.key)

                elif event.type == pygame.KEYUP:
                    keyup(event.key)



            drone.send_rc_control(int(rollspeed), int(pitchspeed), int(throttlespeed), int(yawspeed))

            # data
            text0 = "Battery: {}%".format(drone.get_battery())
            cv2.putText(frame, text0, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text1 = "Acceleration x: {}cm/s^2".format(drone.get_acceleration_x())
            cv2.putText(frame, text1, (5, 680 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

            text2 = "Acceleration y: {}cm/s^2".format(drone.get_acceleration_y())
            cv2.putText(frame, text2, (5, 640 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text3 = "TOF distance: {}cm".format(drone.get_distance_tof())
            cv2.putText(frame, text3, (5, 600 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text4 = "flight time: {}s".format(drone.get_flight_time())
            cv2.putText(frame, text4, (5, 560 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text5 = "temperature: {}C".format(drone.get_temperature())
            cv2.putText(frame, text5, (5, 520 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
 
 
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1/FPS)
        
        
        
        # ----------------------------------------- gate 1 --------------------------------------------
        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎
        

        drone.send_rc_control(0, 0, 0, 0)
        
        drone.send_rc_control(0, 0, 0, 0)

        time.sleep(1)
        
        drone.move_down(50)

        drone.move_forward(350)

        drone.rotate_clockwise(180)
        
        
        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎
        # ----------------------------------------- gate 2 --------------------------------------------
        

        should_stop_gate_2 = False

        while not should_stop_gate_2:
            
            frame = frame_read.frame

            image = frame

            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(gray_image)
            
            mean = None
            d_mean = None
            centers = [0]
            centers.clear()
            distances = [0]
            distances.clear()

            if not detections:
                print("Not detections")
                
                #search()

                #time.sleep(1)

            else:
                for detect in detections:
                    print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))

                    # Calculate distance
                    d = math.sqrt(((detect.corners[0][0]
                                    - detect.corners[2][0]) **
                                   2 + (detect.corners[0][1]
                                        - detect.corners[2][1]) ** 2)/2)
                    distance = (166*100 / d)
                    
                    radius = distance/20
                   
                    mean = calculate_centers_mean_gate(detect.center, detect.tag_id, id_max_2, id_min_2)
                        
                    d_mean = calculate_distances_mean_gate(distance, detect.tag_id, id_max_2, id_min_2)
                
                if mean is not None and d_mean is not None:
                   
                    print("means are not none")

                    image = plot_text(image, "Gate 2", (20, 20))
                    
                    image = draw_center(image, 10, mean)

                    image = plot_text(image, str(d_mean), (20, 50))
                    
                    yawspeed, throttlespeed, pitchspeed = set_velocity(mean, d_mean)
                    rollspeed = 0
                    
                    for event in pygame.event.get():

                        if event.type == pygame.QUIT:
                            should_stop_gate_2 = True

                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_ESCAPE:
                                should_stop_gate_2 = True
                                drone.land()
                            else:
                                keydown(event.key)

                        elif event.type == pygame.KEYUP:
                            keyup(event.key)


                    drone.send_rc_control(int(rollspeed), int(pitchspeed), int(throttlespeed), int(yawspeed))


                    pygame.display.update()

                    # check if the drone is close enough to the center of the apriltag
                    if d_mean < DESIRED_DISTANCE:
                        print("distance < desired distance")
                        
                        drone.send_rc_control(0, 0, 0, 0)

                        should_stop_gate_2 = True
                    

                    # draw the center of the apriltag
                    for detect in detections:                
                        image = draw_center(image, 5, detect.center)
                        
                        # draw the corners of the apriltag
                
                        for corner in detect.corners:

                            image = draw_corners(image, corner)
            
            update()

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    should_stop = True

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                        drone.land()
                    else:
                        keydown(event.key)

                elif event.type == pygame.KEYUP:
                    keyup(event.key)



            drone.send_rc_control(int(rollspeed), int(pitchspeed), int(throttlespeed), int(yawspeed))

            # data
            text0 = "Battery: {}%".format(drone.get_battery())
            cv2.putText(frame, text0, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text1 = "Acceleration x: {}cm/s^2".format(drone.get_acceleration_x())
            cv2.putText(frame, text1, (5, 680 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

            text2 = "Acceleration y: {}cm/s^2".format(drone.get_acceleration_y())
            cv2.putText(frame, text2, (5, 640 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text3 = "TOF distance: {}cm".format(drone.get_distance_tof())
            cv2.putText(frame, text3, (5, 600 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text4 = "flight time: {}s".format(drone.get_flight_time())
            cv2.putText(frame, text4, (5, 560 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text5 = "temperature: {}C".format(drone.get_temperature())
            cv2.putText(frame, text5, (5, 520 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
 
 
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1/FPS)
        
 

 
        
        # ----------------------------------------- gate 2 --------------------------------------------
        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎
        

        drone.send_rc_control(0, 0, 0, 0)
        
        drone.move_down(20)
        
        drone.set_speed(60)

        drone.move_forward(500)

        drone.move_forward(250)
        
        drone.send_rc_control(0, 0, 0, 0)

        drone.rotate_counter_clockwise(85)

        drone.move_forward(500)

        drone.rotate_counter_clockwise(95)

        drone.set_speed(SPEED)

        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎
        # ----------------------------------------- gate 3 --------------------------------------------
        
        
        

        should_stop_gate_3 = False

        while not should_stop_gate_3:
            
            frame = frame_read.frame

            image = frame

            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            detections = detector.detect(gray_image)
            
            mean = None
            d_mean = None
            centers = [0]
            centers.clear()
            distances = [0]
            distances.clear()

            if not detections:
                print("Not detections")
                
                #search()

                #time.sleep(1)

            else:
                for detect in detections:
                    print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))

                    # Calculate distance
                    d = math.sqrt(((detect.corners[0][0]
                                    - detect.corners[2][0]) **
                                   2 + (detect.corners[0][1]
                                        - detect.corners[2][1]) ** 2)/2)
                    distance = (166*100 / d)
                    
                    radius = distance/20
                   
                    mean = calculate_centers_mean_gate(detect.center, detect.tag_id, id_max_3, id_min_3)
                        
                    d_mean = calculate_distances_mean_gate(distance, detect.tag_id, id_max_3, id_min_3)
                
                if mean is not None and d_mean is not None:
                   
                    print("means are not none")

                    image = plot_text(image, "Gate33", (20, 20))
                    
                    image = draw_center(image, 10, mean)

                    image = plot_text(image, str(d_mean), (20, 50))
                    
                    yawspeed, throttlespeed, pitchspeed = set_velocity(mean, d_mean)
                    rollspeed = 0
                    
                    for event in pygame.event.get():

                        if event.type == pygame.QUIT:
                            should_stop_gate_2 = True

                        elif event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_ESCAPE:
                                should_stop_gate_2 = True
                                drone.land()
                            else:
                                keydown(event.key)

                        elif event.type == pygame.KEYUP:
                            keyup(event.key)


                    drone.send_rc_control(int(rollspeed), int(pitchspeed), int(throttlespeed), int(yawspeed))


                    pygame.display.update()

                    # check if the drone is close enough to the center of the apriltag
                    if d_mean < DESIRED_DISTANCE:
                        print("distance < desired distance")
                        
                        drone.send_rc_control(0, 0, 0, 0)
                        
                        drone.move_forward(50)

                        should_stop_gate_3 = True
                    

                    # draw the center of the apriltag
                    for detect in detections:                
                        image = draw_center(image, 5, detect.center)
                        
                        # draw the corners of the apriltag
                
                        for corner in detect.corners:

                            image = draw_corners(image, corner)
            
            update()

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    should_stop = True

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                        drone.land()
                    else:
                        keydown(event.key)

                elif event.type == pygame.KEYUP:
                    keyup(event.key)



            drone.send_rc_control(int(rollspeed), int(pitchspeed), int(throttlespeed), int(yawspeed))

            # data
            text0 = "Battery: {}%".format(drone.get_battery())
            cv2.putText(frame, text0, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text1 = "Acceleration x: {}cm/s^2".format(drone.get_acceleration_x())
            cv2.putText(frame, text1, (5, 680 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

            text2 = "Acceleration y: {}cm/s^2".format(drone.get_acceleration_y())
            cv2.putText(frame, text2, (5, 640 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text3 = "TOF distance: {}cm".format(drone.get_distance_tof())
            cv2.putText(frame, text3, (5, 600 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text4 = "flight time: {}s".format(drone.get_flight_time())
            cv2.putText(frame, text4, (5, 560 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text5 = "temperature: {}C".format(drone.get_temperature())
            cv2.putText(frame, text5, (5, 520 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
 
 
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1/FPS)
        
 
 
        
        # ----------------------------------------- gate 3 --------------------------------------------
        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎
        

        drone.send_rc_control(0, 0, 0, 0)
        
        drone.move_down(20)

        drone.move_forward(400)


        # ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎ ⬇︎
        # ----------------------------------------- land --------------------------------------------
        


        
        
        drone.land()






