import math
import time
import numpy as np
from zmqRemoteApi import RemoteAPIClient

from sensor import Sensor

from constants import *


class Robot:
    def __init__(self, sim, robot_name):
        self._start_time = time.time()
        self.name = robot_name
        self.sim = sim

        self.bug = sim.getObject('/PioneerP3DX')
        self.scanner = sim.getObject('/PioneerP3DX/LaserScanner2D')
        self.goal = sim.getObject('/Goal')
        self.sensor = Sensor()

    def _set_movement(self, left, right):
        motorLeft = sim.getObject("./leftMotor")
        motorRight = sim.getObject("./rightMotor")
        self.sim.setJointTargetVelocity(motorLeft, left)
        self.sim.setJointTargetVelocity(motorRight, right)

    def rotate(self):
        motorLeft = sim.getObject("./leftMotor")
        motorRight = sim.getObject("./rightMotor")
        self.sim.setJointTargetVelocity(motorLeft, 0)
        self.sim.setJointTargetVelocity(motorRight, 0)

    def start(self):
        """
        Старт работы робота
        """
        while 1:
            self._set_movement(2, 2)

            states = [self.sensor.left_sector.status, self.sensor.mid_sector.status, self.sensor.right_sector.status]
            print(states)

            if any(map(lambda x: x.value, states)):
                # Нечёткая логика начинается здесь
                ...
            else:
                # Тут чёткая наводка на цель
                ...

            # distance = self.sensor.min_dist
            # print(distance)
            # if (not np.isnan(distance)) and (distance < ROBOT_STOP_DISTANCE):
                # self._set_movement(0, 0, 0)
                # break


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    sim.startSimulation()

    robot = Robot(sim, 'PioneerP3DX')
    robot.rotate()
    # robot.start()

