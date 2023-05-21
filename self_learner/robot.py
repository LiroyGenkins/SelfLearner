import math
import time
from coppeliasim_api.zmqRemoteApi import RemoteAPIClient

import numpy as np

from navigation.navigation import Navigator
from planner.planner import Planner

import constants as const


def get_wheel_coefs(turn):
    if turn < 0:
        left = 1 / abs(turn)
        right = 1
    elif turn > 0:
        left = 1
        right = 1 / turn
    else:
        left = 1
        right = 1

    return left, right


class Robot:
    def __init__(self, sim, robot_name):
        self.target_distance = None
        self._start_time = time.time()
        self.name = robot_name
        self.sim = sim

        self.robot_handle = sim.getObject(f"./{const.ROBOT_NAME}")
        self._target_handle = sim.getObject("./Goal")
        self._left_motor_handle = sim.getObject("./leftMotor")
        self._right_motor_handle = sim.getObject("./rightMotor")

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
        self.sim.setJointTargetVelocity(self._left_motor_handle, left)
        self.sim.setJointTargetVelocity(self._right_motor_handle, right)

    # TODO: методы для движения прямо, движения с поворотом, поворота на месте и т.д.
    def rotate(self):
        self.sim.setJointTargetVelocity(self._left_motor_handle, 0)
        self.sim.setJointTargetVelocity(self._right_motor_handle, 0)

        while 1:
            relative_position = sim.getObjectPosition(self._target_handle, self.robot_handle)
            angle = math.atan2(relative_position[1], relative_position[0]) * 180 / math.pi
            print(angle)
            if abs(angle) < 0.3:
                break

            if angle > 0:
                self.sim.setJointTargetVelocity(self._left_motor_handle, -1)
                self.sim.setJointTargetVelocity(self._right_motor_handle, 1)
            elif angle < 0:
                self.sim.setJointTargetVelocity(self._left_motor_handle, 1)
                self.sim.setJointTargetVelocity(self._right_motor_handle, -1)
            self.sim.setJointTargetVelocity(self._left_motor_handle, 0)
            self.sim.setJointTargetVelocity(self._right_motor_handle, 0)

    def move(self):
        self.sim.setJointTargetVelocity(self._left_motor_handle, 1)
        self.sim.setJointTargetVelocity(self._right_motor_handle, 1)
        while 1:
            distance = self.sim.checkDistance(self.robot_handle, self._target_handle, 0)
            print(distance)
            if distance[1][6] == 0.0:
                self.sim.setJointTargetVelocity(self._left_motor_handle, 0)
                self.sim.setJointTargetVelocity(self._right_motor_handle, 0)
                break

    def start(self):
        """
        Старт работы робота
        """

        # Начальный поворот на цель и установка флага объезда препятствия
        # prev_value = 55
        left_velocity = const.ROBOT_SPEED
        right_velocity = const.ROBOT_SPEED
        fuzzy_flag = False
        # robot.rotate()
        while 1:
            self.target_distance = self.sim.checkDistance(self.robot_handle, self._target_handle, 0)[1][6]
            states = (self._navigator.left_sector.status,
                      self._navigator.mid_sector.status,
                      self._navigator.right_sector.status)
            print(states)

            # left_sec, mid_sec, right_sec = states

            # if any([mid_sec, left_sec, right_sec]):
                # Нечёткая логика начинается здесь
            print("Нечотко")
            decision = self._planner.make_decision(self._navigator)
            # print(self._navigator.target_angle)
            print(decision)

            # wheel_coefs = {0: (0.6, 1), 1: (0.7, 1), 2: (0.95, 1),
            #                3: (1, 0.95), 4: (1, 0.7), 5: (1, 0.6)}

            left, right = get_wheel_coefs(decision)
            # if self._navigator.mid_sector.status != prev_value:  # or (not mid_sec):
                # left, right = 1, 1
            left_velocity = left * const.ROBOT_SPEED
            right_velocity = right * const.ROBOT_SPEED
                # fuzzy_flag = True


            prev_value = self._navigator.mid_sector.status
            print(prev_value)

            # elif fuzzy_flag and not mid_sec:
            #     fuzzy_flag = False
            #     robot.rotate()
            # else:
            #     # Тут чёткая наводка на цель
            #     print("Чотко")
            #     left_velocity = const.ROBOT_SPEED
            #     right_velocity = const.ROBOT_SPEED

            self._set_movement(left_velocity, right_velocity)

            distance = self._navigator.min_dist
            end = time.time()
            if (not np.isnan(distance)) and (distance < const.ROBOT_STOP_DISTANCE):
                self._set_movement(0, 0)
                break
            if target_distance[1][6] == 0.0:
                self._set_movement(0, 0)
                break
            if end - start > const.CREATURE_LIFETIME:
                self._set_movement(0, 0)
                break

        print(distance)
        self.sim.setObjectOrientation(self.robot_handle, -1, start_orientation)
        self.sim.setObjectPosition(self.robot_handle, -1, start_position)



if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    sim.startSimulation()

    robot = Robot(sim, const.ROBOT_NAME)

    start_orientation = robot.sim.getObjectOrientation(robot.robot_handle, -1)
    start_position = robot.sim.getObjectPosition(robot.robot_handle, -1)

    # Генетический алгоритм:
    for i in range(const.NUM_CREATURES):
        start = time.time()

        robot.start()

        end = time.time() - start
        print(end)
        print(robot.target_distance)

        robot.sim.setObjectOrientation(robot.robot_handle, -1, start_orientation)
        robot.sim.setObjectPosition(robot.robot_handle, -1, start_position)




