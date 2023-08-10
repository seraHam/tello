from djitellopy import Tello
import cv2, math, time

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()
time.sleep(1)
while True:
    # In reality you want to display frames in a seperate thread. Otherwise
    #  they will freeze while the drone moves.
    # 在实际开发里请在另一个线程中显示摄像头画面，否则画面会在无人机移动时静止
    img = frame_read.frame
    cv2.imshow("drone", img)

    data_read = tello.get_current_state()
    print (data_read)
    print ("VGX PRINTOUT", data_read["vgx"])
    print ("did you reach?")
    time.sleep(1)
    key = cv2.waitKey(1) & 0xff
    if key == 27: # ESC
        break
    elif key == ord('w'):
        tello.move_forward(10)
    elif key == ord('s'):
        tello.move_back(10)
    elif key == ord('a'):
        tello.move_left(10)
    elif key == ord('d'):
        tello.move_right(10)
    elif key == ord('e'):
        tello.rotate_clockwise(10)
    elif key == ord('q'):
        tello.rotate_counter_clockwise(10)
    elif key == ord('r'):
        tello.move_up(10)
    elif key == ord('f'):
        tello.move_down(10)

tello.land()