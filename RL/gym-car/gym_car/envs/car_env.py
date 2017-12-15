import numpy as np
from PIL import Image
import socket
import struct
from io import BytesIO
import base64

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import cv2
# pre-process input image
# turn gray
# thresholds for vertical gradient + gradient magnitude + gradient direction
# output is 1 layer binary image
def pre_process(image):
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

    abs_sobel =  np.absolute(sobel_y)
    if (np.max(abs_sobel) < 1e-5):
        print("error!")
        return np.zeros([160,320,1]).astype(np.uint8) 
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

    gradmag = np.sqrt(sobel_x**2 + sobel_y**2)
    scale_factor = np.max(gradmag)/255
    if (scale_factor < 1e-5):
        print("error!")
        return np.zeros([160,320,1]).astype(np.uint8) 
    gradmag = (gradmag/scale_factor).astype(np.uint8) 

    absgraddir = np.arctan2(np.absolute(sobel_y), np.absolute(sobel_x))

    binary_output = np.zeros_like(gradmag)

    binary_output[(gradmag >= 50) & (gradmag <= 120)
                  & (absgraddir >= 0.7) & (absgraddir <= 2)
                  & (scaled_sobel >= 50) & (scaled_sobel <= 120)] = 255

    binary_output[:70,:] = 0
    binary_output[-50:,:] = 0
    binary_output = np.uint8(binary_output)

    return binary_output.reshape(160,320,1)

class SimplePIController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.set_point = 0.
        self.error = 0.
        self.integral = 0.

    def set_desired(self, desired):
        self.set_point = desired

    def update(self, measurement):
        # proportional error
        self.error = self.set_point - measurement

        # integral error
        self.integral += self.error

        return self.Kp * self.error + self.Ki * self.integral


class CarEnv(gym.Env):

    MaxSteps = 30

    @property
    def action_space(self):
        return spaces.Box(low=-1, high=1, shape=(1,))

    @property
    def observation_space(self):
        return spaces.Box(low=0, high=255, shape=(160, 320, 1))

    def __init__(self):
        self.steps = 0

        self.controller = SimplePIController(0.1, 0.002)
        self.set_speed = 15
        self.controller.set_desired(self.set_speed)

        self.speed = 0

        # start TCP server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        TCP_SERVER_IP = "127.0.0.1"
        TCP_SERVER_PORT = 15000
        self.sock.bind((TCP_SERVER_IP, TCP_SERVER_PORT))
        self.sock.listen(5)
        self.connection, addr = self.sock.accept()
        print("connected to ", self.connection)

    def _step(self, action):

        #execute action
        self.steps += 1        
        #steering_angle = (float(action)-1.)/4.
        steering_angle = float(action)/4.
        throttle = self.controller.update(self.speed);
        restart = 0
        self.connection.send(bytearray(struct.pack("ff?", steering_angle, throttle, bool(restart))))

        #return new state and reward
        readBuffer = self.connection.recv(64000)
        cte, self.speed = struct.unpack('ff',readBuffer[:8])
        image = Image.open(BytesIO(base64.b64decode(readBuffer[8:])))
        observation = pre_process(np.asarray(image)) #return image -> input to actor policy network
        reward = -np.log(1+abs(cte))

        #check if episode is complete
        if self.steps >= CarEnv.MaxSteps:
            done = True
        else:
            done = False
        
        return observation, reward, done, {}

    def _reset(self):

        #execute zero action
        steering_angle = 0.
        throttle = 0.
        restart = 1
        self.connection.send(bytearray(struct.pack("ff?", steering_angle, throttle, bool(restart))))

        #return new state and reward
        readBuffer = self.connection.recv(64000)
        cte, self.speed = struct.unpack('ff',readBuffer[:8])
        image = Image.open(BytesIO(base64.b64decode(readBuffer[8:])))
        observation = pre_process(np.asarray(image))
        reward = -np.log(1+abs(cte))

        self.steps = 0
        
        return observation
        
    def _render(self, mode='human', close=False):
        return

    def _close(self):
        self.steps = 0
        self.sock.close()


