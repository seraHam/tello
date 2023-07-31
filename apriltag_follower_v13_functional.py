 

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
FPS = 20 #frame per second

LINE_LENGTH = 5 #length of the crosshair lines
ENTER_COLOR = (255, 0, 0) #color of the crosshair
CORNER_COLOR = (0, 255, 180) #color of the corners

ddesired = 1 #(meters) desired distance from the tag
yawfactor = 5 #yaw speed factor: can be changed in order to make the drone rotate faster or slower
throttlefactor = 5 #throttle speed factor: can be changed in order to make the drone go up and down faster or slower

tello = Tello() #initialize the tello drone, as tello
tello.connect() #connect to the drone's wifi

tello.streamoff() #in case the streaming is on, turn it off
tello.streamon() #turn on the streaming


# draw a crosshair
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image, (center[0] - LINE_LENGTH, center[1]), (center[0] + LINE_LENGTH, center[1]), color, 3) #draw a horizontal line
    image = cv2.line(image, (center[0], center[1] - LINE_LENGTH), (center[0], center[1] + LINE_LENGTH), color, 3) #draw a vertical line

    return image #return the image with the crosshair drawn on it


# draw a circle
def plotCircle(image, center, color, radius):
    
    center = (int(center[0]), int(center[1]))
    image = cv2.circle(image, center, int(radius), color, 2) # draw a circle
    
    return image #return the image with the circle drawn on it


# insert text
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4) # shift the text a little bit down and to the right so that you can put a crosshair or a circle an then show some information
    
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3) #return the image with the text on it



detector = apriltag.Detector() #initialize the detector



class FrontEnd(object):

    
    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720]) # size can be changed, but thhen all the coordinates should also be changed

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 20
        
        # set the rc control to false, it will be turned to true once the drone takes off
        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)


    def run(self):

        self.tello.connect() #connect to the drone's wifi
        self.tello.set_speed(self.speed) # set the drone's speed

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()

        frame_read = self.tello.get_frame_read() #get the frame from the drone's camera

        should_stop = False #variable that will be used to stop the program
        while not should_stop: #while the program is running
            if frame_read.stopped: #if the video streaming stops
                break #break the loop

            self.screen.fill([0, 0, 0]) #fill the screen with black color, so that the previous frame is not shown anymore

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
                
                
                # calculate the mean of all the apriltags centers
                # and the mean of all the distances
                

                centerx = 0
                centery = 0
                z = 0

                for detect in detections:
                    centerx = centerx + detect.center[0]
                    centery = centery + detect.center[1]
                    distance = math.sqrt(((detect.corners[0][0]
                                    - detect.corners[2][0]) **
                                   2 + (detect.corners[0][1]
                                        - detect.corners[2][1]) ** 2)/2)
                    z = z + ((157.8 / distance)+0.0)

                centerx = centerx / len(detections)
                centery = centery / len(detections)
                z = z / len(detections)

                # plot the mean of all the apriltags centers
                # and the mean of all the distances
                # and the velocity
                # and the yaw velocity
                # and the throttle velocity

                image = plotPoint(image, (centerx, centery), CENTER_COLOR)
                image = plotText(image, (centerx, centery + 20), CENTER_COLOR, "z: " + str(z))
                image = plotText(image, (centerx, centery + 40), CENTER_COLOR, "vx: " + str(self.for_back_velocity))
                image = plotText(image, (centerx, centery + 60), CENTER_COLOR, "vy: " + str(self.left_right_velocity))
                image = plotText(image, (centerx, centery + 80), CENTER_COLOR, "vz: " + str(self.up_down_velocity))
                image = plotText(image, (centerx, centery + 100), CENTER_COLOR, "vyaw: " + str(self.yaw_velocity))

                # set the velocity

                self.setvelocity((centerx, centery), z)

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
            
            text3 = "height: {}cm".format(self.tello.get_height())
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

















