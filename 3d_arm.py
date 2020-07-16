'''press 'c' to clear and 'e' to exit'''


from math import *
import time
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D

class Actuator():
    def __init__(self,name):
        self.name = name
        self.goal_angle = 5
        self.current_angle = 0
        self.speed = 0
        self.dir = 0

        self.pterm = 0
        self.iterm = 0
        self.dterm = 0
        self.kp = 0.01
        self.ki = 0
        self.kd = 0

        self.error = 1
        self.previous_error = 1
        self.max_speed = 10
        self.delta_t = 1

    def run(self,goal_angle):
        self.goal_angle = goal_angle
        self.speed = self.pid()
        pos = self.goal_angle < self.current_angle
        self.dir = (-1)**pos
        self.current_angle += self.dir * self.speed
        # print(f"{self.name}'s error : {self.error}")
    
    def pid(self):
        self.error = self.goal_angle - self.current_angle 

        self.pterm = self.max_speed*(1-exp(-self.kp * self.error**2))
        self.iterm += self.ki * self.error * self.delta_t
        self.dterm = self.kd * (self.error - self.previous_error) / self.delta_t

        if self.iterm > self.max_speed:  # catering intergral windup
            self.iterm = self.max_speed
        elif self.iterm < 0:  
            self.iterm = 0

        speed = self.pterm + self.iterm +self.dterm

        if speed > self.max_speed:
            speed = self.max_speed
        elif speed < 0:
            speed = self.pterm

        return speed

    def display_state(self):
        print('=========================================================================')
        print(f"                         {self.name}")
        print(f'Goal Angle: {self.goal_angle}     Current Angle: {self.current_angle}')
        print(f'Error: {self.error}     Speed: {self.speed}')
# =========================================================================================

class Arm():
    base = Actuator('Base')
    shoulder = Actuator('Shoulder')
    elbow = Actuator('Elbow')

    def __init__(self):
        self.link1 = 15
        self.link2 = 14
        self.end_position = [0,0,0]
        self.goal = [0,0,0]
        self.elbow_point = [0,0,0]
        self.error = 1

    def DistanceFromGoal(self):
        [gx,gy,gz],[x,y,z] = self.goal, self.end_position
        dist = sqrt((gx-x)**2 + (gy-y)**2 + (gz-z)**2)
        # print('overall error = ',dist)
        return dist        
    
    def update(self):
        theta = self.base.current_angle * pi/180
        alpha = self.shoulder.current_angle * pi/180
        beta = self.elbow.current_angle * pi/180

        ax = self.link1*cos(alpha)*cos(theta)
        ay = self.link1*cos(alpha)*sin(theta)
        az = self.link1*sin(alpha) 

        R = sqrt(self.link1*self.link1 + self.link2*self.link2 - 2*self.link1*self.link2*cos(beta))
        bz = az + self.link2*sin(alpha + beta -pi)
        alpha2 = asin(bz/R)
        bx = R*cos(alpha2)*cos(theta)
        by = R*cos(alpha2)*sin(theta)

        self.end_position = [bx,by,bz]
        self.elbow_point = [ax,ay,az]

        return self.end_position, self.elbow_point
    
    def InverseKinematics(self):
        x,y,z = self.goal

        try :
            beta = acos((self.link1*self.link1 + self.link2*self.link2 - x*x - y*y - z*z)/(2*self.link1*self.link2))
            alpha1 = asin(self.link2*sin(beta)/sqrt(x*x + y*y + z*z))
            alpha2 = asin(z/sqrt(x*x + y*y + z*z))
            alpha = alpha1 + alpha2
            flag1 = x > 0
            flag2 = y >= 0
            if flag1:
                theta = (1 + (-1)**flag2) * pi + atan(y/x)
            else:
                theta = pi + atan(y/x)

            angles = {}
            angles['theta'] = theta*180/pi
            angles['alpha1'] = alpha1*180/pi
            angles['alpha2'] = alpha2*180/pi
            angles['alpha'] = alpha*180/pi
            angles['beta'] = beta*180/pi

            return angles
        except:
            print('Goal Unreachable !')
            return None

    def prepare_and_run(self,goal):
        global history
        self.goal = goal
        angles = self.InverseKinematics()

        if angles is not None: 
            while(self.error > 0.5):
                self.error = self.DistanceFromGoal()
                self.base.run(angles['theta'])
                self.base.display_state()
                self.shoulder.run(angles['alpha'])
                self.shoulder.display_state()
                self.elbow.run(angles['beta'])
                self.elbow.display_state()
                end_position , elbow_point = self.update()
                history[0].append(end_position[0])
                history[1].append(end_position[1])
                history[2].append(end_position[2])
                plot(end_position, elbow_point, goal)
            time.sleep(2)
# ============================================================================================================
     
def plot(end_position, elbow_point, goal):
    global history

    ax,ay,az = elbow_point
    bx,by,bz = end_position

    ax1.clear()

    ax1.plot(history[0],history[1],history[2],'c--', label = 'Path')

    ax1.plot([0,ax,bx],[0,ay,by],[0,az,bz],'k-',label = 'Link')
    ax1.plot([0,ax,bx],[0,ay,by],[0,az,bz],'ro',label = 'Joint')
    ax1.plot([0,0],[0,0],[0,-0.2],'b-',linewidth = 10)

    ax1.plot([goal[0]],[goal[1]],[goal[2]],'go',label = 'Goal')

    ax1.legend()
    ax1.set_xlim(-16,16)
    ax1.set_ylim(-16,16)
    ax1.set_zlim(-10,16)
    ax1.set_title('3D Robot Arm Simulator')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')

    plt.show()
    plt.pause(dt)

def on_key(event):
    global history, flag
    if event.key == 'c' or event.key == 'C': # press c to clear path history
        history = [[],[],[]]
    elif event.key == 'e' or event.key == 'E': # press e to end simulation
        plt.close()
        flag = 0

plt.ion()
dt = 0.0001
tick = 0
tock = 0
flag = 1
history = [[],[],[]]
fig = plt.figure()
ax1 = fig.add_subplot(111,projection = '3d')
fig.canvas.mpl_connect('key_press_event',on_key) 
# demo_animation()
flag = 1
while(flag):
    # g = list(map(float,input('x,y,z : ').split(',')))
    g = np.random.random(3)*25 - 10
    print(g)
    robot = Arm()
    robot.prepare_and_run(g)
    # flag = int(input('continue? (0/1): '))