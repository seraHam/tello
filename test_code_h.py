from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import matplotlib.pyplot as plt
# Speed of the drone
# 无人机的速度
S = 60
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
# pygame窗口显示的帧数
# 较低的帧数会导致输入延迟，因为一帧只会处理一次输入信息
FPS = 30

goal_loc = [2.0, 0.0, 2.0]

class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations (yaw)
            - W and S: Up and down.

        保持Tello画面显示并用键盘移动它
        按下ESC键退出
        操作说明：
            T：起飞
            L：降落
            方向键：前后左右
            A和D：逆时针与顺时针转向
            W和S：上升与下降

    """

    def __init__(self):
        # Init pygame
        # 初始化pygame
        pygame.init()

        # Creat pygame window
        # 创建pygame窗口
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        # 初始化与Tello交互的Tello对象
        self.tello = Tello()

        # Drone velocities between -100~100
        # 无人机各方向速度在-100~100之间
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 100

        self.send_rc_control = False

        # create update timer
        # 创建上传定时器
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)

    def run(self):

        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        # 防止视频流已开启。这会在不使用ESC键退出的情况下发生。
        self.tello.streamoff()
        self.tello.streamon()
        # self.tello.set_video_direction(0)

        frame_read = self.tello.get_frame_read()

        should_stop = False
        x = np.zeros(1)
        y = np.zeros(1)
        x_pl = np.array([] * 0).reshape(0, 1)
        y_pl = np.array([] * 0).reshape(0, 1)
        z_pl = np.array([] * 0).reshape(0, 1)
        vx_pl = np.array([]* 0).reshape(0, 1)
        vy_pl = np.array([] * 0).reshape(0, 1)
        vz_pl = np.array([] * 0).reshape(0, 1)
        pos_ax = np.array([] * 0).reshape(0, 1)
        pos_ay = np.array([] * 0).reshape(0, 1)
        pos_az = np.array([] * 0).reshape(0, 1)
        pos_baro = np.array([] * 0).reshape(0, 1)
        ax_dt2 = 0
        ay_dt2 = 0
        az_dt2 = 0
        dt = 1 / 300
        pos_baro_init = 0

        while not should_stop:

            # data_read = self.tello.get_current_state()
            vx = self.tello.get_speed_x()
            vy = self.tello.get_speed_y()
            vz = self.tello.get_speed_z()

            baro_h = self.tello.get_barometer()

            # print('vx: ', vx, ' vy: ', vy, ' vz:', vz)
            

            # print(baro_h)
            # print(data_read)

            # vx = np.array([data_read["vgx"]])
            # vy = np.array([data_read["vgy"]])
            # for key in data_read:
                # print(key, data_read[key])

            # time.sleep(1)
            h = self.tello.get_height()
            x = vx
            y = vy
            z = h / 100

            print(z)
            
            ax_dt2_temp = self.tello.get_acceleration_x() / 100
            
            ax_dt2 = (ax_dt2 + ax_dt2_temp) / 2
             

            ay_dt2 = (ay_dt2 + self.tello.get_acceleration_y() /100) / 2

            pos_ax = np.append(pos_ax, np.array([ax_dt2]))
            pos_ay = np.append(pos_ay, np.array([ay_dt2]))

            ax_dt2 = ax_dt2_temp

            az_dt2 = (az_dt2 - self.tello.get_acceleration_z() / 100 - 10.14) / 2

            pos_az = np.append(pos_az, -1 * np.array([az_dt2]))
            
            if pos_baro_init == 0:
                pos_baro_init = baro_h

            pos_baro_current = np.array([(baro_h- pos_baro_init) / 100.0])
            pos_baro = np.append(pos_baro, pos_baro_current) 

            # print('x: ', x, ' y: ', y)

            x_pl = np.append(x_pl, np.array([x]).reshape(1, 1))
            y_pl  = np.append(y_pl, np.array([y]).reshape(1, 1))
            z_pl = np.append(z_pl, np.array([z]).reshape(1, 1))
            vx_pl = np.append(vx_pl, np.array([vx]).reshape(1, 1))
            vy_pl= np.append(vy_pl, np.array([vy]).reshape(1, 1))
            vz_pl = np.append(vz_pl, np.array([vz]).reshape(1, 1))

            vel_command_x = int(100 * (goal_loc[0] - x))
            vel_command_y = int(100 * (goal_loc[1] - y))
            up_down_velocity = int(100 * (goal_loc[2] - pos_baro_current))
            for_back_velocity = vel_command_x * 0 
            left_right_velocity = vel_command_y * 0
            yaw_velocity = 0

            # print('vx_cmd: ', vel_command_x, ' vy_cmd: ', vel_command_y)
            # print('ax: ', ax_dt2, ' ay: ', ay_dt2, ' az: ', self.tello.get_acceleration_z() / 100)
            
            # time.sleep(0.01)

            if np.linalg.norm([pos_baro_current- goal_loc[2]]) < 0.1: #np.linalg.norm([x - goal_loc[0], y - goal_loc[1]]) < 0.1:
                self.keyup(pygame.K_l)
                should_stop = True
                break
            
            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    should_stop = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == pygame.KEYUP:
                    self.keyup(event.key)
                if self.send_rc_control == True:
                    self.tello.send_rc_control(left_right_velocity, for_back_velocity,
                up_down_velocity, yaw_velocity)
            

            if frame_read.stopped:
                break

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame
            # battery n. 电池
            text = "Battery: {}%".format(self.tello.get_battery())
            cv2.putText(frame, text, (5, 720 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()
            time.sleep(1/1000)
            # time.sleep(1 / FPS)

        # fig = plt.figure()
        fig, ax = plt.subplots(3, 3)
        ax0 = ax[0, 0]
        ax0.plot(x_pl)
        ax1 = ax[1, 0]
        ax1.plot(vx_pl)
        ax2 = ax[0, 1]
        ax2.plot(y_pl)
        ax3 = ax[1, 1]
        ax3.plot(vy_pl)
        ax4 = ax[2, 0]
        ax4.plot(pos_ax)
        ax5 = ax[2, 1]
        ax5.plot(pos_ay)
        ax6 = ax[0, 2]
        ax6.plot(z_pl)
        ax7 = ax[1, 2]
        ax7.plot(pos_baro)
        ax8 = ax[2, 2]
        ax8.plot(pos_az)
        
        plt.savefig('./traj.png')
        # plot x, y data
        # ax = plt.subplot(1, 1, 1)
        # ax.plot(x_pl[:, 0], x_pl[:, 1])

        # plt.savefig('./traj.png')
        # Call it always before finishing. To deallocate resources.
        # 通常在结束前调用它以释放资源
        self.tello.end()

        
    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key

        基于键的按下上传各个方向的速度
        参数：
            key：pygame事件循环中的键事件
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

        基于键的松开上传各个方向的速度
        参数：
            key：pygame事件循环中的键事件
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

    def update(self):
        """ Update routine. Send velocities to Tello.

            向Tello发送各方向速度信息
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