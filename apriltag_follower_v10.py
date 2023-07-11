#worksss!!!
# and faast!!
# how it works:
# you can controle it manually
# if it detects an apriltags: it shows on the screen the center, the distance
# and the position
# it moves alltogether (changing the velocities)
# 

import cv2
import apriltag
from djitellopy import Tello 
import av
import time
import pygame
import numpy as np
import math

fermo = 0
comencare = 1

S = 60 #speed
FPS = 20

LINE_LENGTH = 5
CENTER_COLOR = (255, 0, 0)
CORNER_COLOR = (0, 255, 180)

ddesired = 1 #(meters)
yawfactor = 5
throttlefactor = 5

tello = Tello()
tello.connect()

tello.streamoff()
tello.streamon()

### Some utility functions to simplify drawing on the camera feed
# draw a crosshair
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image, (center[0] - LINE_LENGTH, center[1]), (center[0] + LINE_LENGTH, center[1]), color, 3)
    image = cv2.line(image, (center[0], center[1] - LINE_LENGTH), (center[0], center[1] + LINE_LENGTH), color, 3)

    return image


# plot a little text

def plotCircle(image, center, color, radius):
    
    center = (int(center[0]), int(center[1]))
    image = cv2.circle(image, center, int(radius), color, 2)

    #cv2.circle(image, center_coordinates, radius, color, thickness)
    
    return image



def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)



# setup and the main loop
detector = apriltag.Detector()




class FrontEnd(object):

    
    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 20

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    def run(self):

        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:
            if frame_read.stopped:
                break

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame
        
            image = frame
            grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(grayimg)

            if not detections:
                print("Nothing")
                
                global fermo

                if (fermo == 1):
                    
                    fermo = 0

                    self.for_back_velocity = 0
                    self.left_right_velocity = 0
                    self.up_down_velocity = 0
                    self.yaw_velocity = 0
             

            else:
                for detect in detections:
                    
                    global comencare

                    if (comencare == 1):
                        comencare = 0
                        self.tello.takeoff()
                        self.send_rc_control = True

                    else:

                        fermo = 1

                        print("tag_id: %s, center: %s" % (detect.tag_id, detect.center))

                        # Calculate distance
                        distance = math.sqrt(((detect.corners[0][0]
                                        - detect.corners[2][0]) **
                                       2 + (detect.corners[0][1]
                                            - detect.corners[2][1]) ** 2)/2)
                        z = ((157.8 / distance)+0.0)

                        # Display distance
                        #image = plotText(image, detect.center, CENTER_COLOR, f"Distance: {z:.2f}")
                        
                        image = plotText(image, detect.center, CENTER_COLOR, z)

                        #show circle: size depends on the distance

                        radius = (z*20)

                        image = plotCircle(image, detect.center, CENTER_COLOR, radius)
                    

                        # new thing --------------------------------:
                        
                        self.setvelocity(detect.center, z)

                        self.update()


                for corner in detect.corners:
                    image = plotPoint(image, corner, CORNER_COLOR)
                    
                    image = plotText(image, corner, CORNER_COLOR, corner)


            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                        
                        # new thing ---------------------------:
                        self.tello.land()
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)



            # data
            text0 = "Battery: {}%".format(self.tello.get_battery())
            cv2.putText(frame, text0, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text1 = "Acceleration x: {}cm/s^2".format(self.tello.get_acceleration_x())
            cv2.putText(frame, text1, (5, 680 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

            text2 = "Acceleration y: {}cm/s^2".format(self.tello.get_acceleration_y())
            cv2.putText(frame, text2, (5, 640 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text3 = "TOF distance: {}cm".format(self.tello.get_distance_tof())
            cv2.putText(frame, text3, (5, 600 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text4 = "flight time: {}s".format(self.tello.get_flight_time())
            cv2.putText(frame, text4, (5, 560 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            
            text5 = "temperature: {}C".format(self.tello.get_temperature())
            cv2.putText(frame, text5, (5, 520 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)
            

            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. To deallocate resources.
        self.tello.end()
        
    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key

        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw counter clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw clockwise velocity
            self.yaw_velocity = S

    
    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key

        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        
        
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            not self.tello.land()
            self.send_rc_control = False

     
    def setvelocity(self, center, z):
        

        center = (int(center[0]), int(center[1]))
        
        yawspeed = (int(center[0]-480)/yawfactor)

        throttlespeed = -(int(center[1]-360)/throttlefactor)

        pitchspeed = ((z*100) - (ddesired*100))/1.5

        if (throttlespeed < -50):
            not self.tello.land() 
            self.send_rc_control = False

        self.for_back_velocity = int(pitchspeed)
        
        self.left_right_velocity = 0
        
        self.up_down_velocity = int(throttlespeed)
        
        self.yaw_velocity = int(yawspeed)
           



   

    def update(self):
        """ Update routine. Send velocities to Tello.

        """
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,
                self.up_down_velocity, self.yaw_velocity)


def main():
    frontend = FrontEnd()

    # run frontend

    frontend.run()


if __name__ == '__main__':
    main()














