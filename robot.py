import time
import numpy as np
from zmqRemoteApi import RemoteAPIClient

from sensing import Navigator
from planner import Planner

import constants as const


class Robot:
    def __init__(self, sim, robot_name):
        self._start_time = time.time()
        self.name = robot_name
        self.sim = sim

        self._navigator = Navigator()
        self._planner = Planner()

    # def _set_movement(self, forward_back_vel, left_right_vel, rotation_vel):
    #     """
    #     Установка движения робота
    #
    #     Parameters
    #     ----------
    #     forward_back_vel : float
    #         Скорость в направлении вперёд-назад
    #     left_right_vel : float
    #         Скорость в направлении влево-вправо
    #     rotation_vel : float
    #         Скорость поворота
    #     """
    #     wheel_joints = [self.sim.getObject('/' + self.name + '/rollingJoint_fl'),
    #                     self.sim.getObject('/' + self.name + '/rollingJoint_rl'),
    #                     self.sim.getObject('/' + self.name + '/rollingJoint_rr'),
    #                     self.sim.getObject('/' + self.name + '/rollingJoint_fr')]
    #     self.sim.setJointTargetVelocity(wheel_joints[0], -forward_back_vel - left_right_vel - rotation_vel)
    #     self.sim.setJointTargetVelocity(wheel_joints[1], -forward_back_vel + left_right_vel - rotation_vel)
    #     self.sim.setJointTargetVelocity(wheel_joints[2], -forward_back_vel - left_right_vel + rotation_vel)
    #     self.sim.setJointTargetVelocity(wheel_joints[3], -forward_back_vel + left_right_vel + rotation_vel)

    def _set_movement(self, left, right):
        left_motor = sim.getObject("./leftMotor")
        right_motor = sim.getObject("./rightMotor")
        self.sim.setJointTargetVelocity(left_motor, left)
        self.sim.setJointTargetVelocity(right_motor, right)

    # TODO: методы для движения прямо, движения с поворотом, поворота на месте и т.д.
    def rotate(self):
        left_motor = sim.getObject("./leftMotor")
        right_motor = sim.getObject("./rightMotor")
        self.sim.setJointTargetVelocity(left_motor, 0)
        self.sim.setJointTargetVelocity(right_motor, 0)

    def start(self):
        """
        Старт работы робота
        """
        while 1:
            self._set_movement(const.ROBOT_SPEED, const.ROBOT_SPEED)

            states = [self._navigator.left_sector.status,
                      self._navigator.mid_sector.status,
                      self._navigator.right_sector.status]
            # print(states)
            # print(self._planner.make_decision(self._navigator.target_side,
            #                                   self._navigator.left_sector.status,
            #                                   self._navigator.mid_sector.status,
            #                                   self._navigator.right_sector.status))

            print(self._navigator.target_side)

            if any(map(lambda x: x.value, states)):
                # Нечёткая логика начинается здесь
                ...
            else:
                # Тут чёткая наводка на цель
                ...

            # distance = self._navigator.min_dist
            # if (not np.isnan(distance)) and (distance < ROBOT_STOP_DISTANCE):
            #     self._set_movement(0, 0, 0)
            #     break


if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    sim.startSimulation()

    robot = Robot(sim, const.ROBOT_NAME)
    robot.start()
