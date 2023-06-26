import math
import time
from coppeliasim_api.zmqRemoteApi import RemoteAPIClient

import numpy as np

from navigation.navigation import Navigator
import planner
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
    def __init__(self, sim, robot_name, training=False):
        self.target_distance = None
        self._start_time = time.time()
        self.name = robot_name
        self.sim = sim

        self.sum_rot = 0

        self.robot_handle = sim.getObject(f"./{const.ROBOT_NAME}")
        self._target_handle = sim.getObject("./Goal")
        self._left_motor_handle = sim.getObject("./leftMotor")
        self._right_motor_handle = sim.getObject("./rightMotor")

        self.prev_orientation = self.sim.getObjectOrientation(self.robot_handle, -1)

        self._navigator = Navigator()
        self.planner = Planner()

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
            # print(angle)
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
            # print(distance)
            if distance[1][6] == 0.0:
                self.sim.setJointTargetVelocity(self._left_motor_handle, 0)
                self.sim.setJointTargetVelocity(self._right_motor_handle, 0)
                break

    def start(self):
        """
        Старт работы робота
        """
        start = time.time()

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
            # print(states)

            # left_sec, mid_sec, right_sec = states

            # if any([mid_sec, left_sec, right_sec]):
            # Нечёткая логика начинается здесь
            # print("Нечотко")
            decision = self.planner.make_decision(self._navigator)
            # print(self._navigator.target_angle)
            # print(decision)

            # wheel_coefs = {0: (0.6, 1), 1: (0.7, 1), 2: (0.95, 1),
            #                3: (1, 0.95), 4: (1, 0.7), 5: (1, 0.6)}

            left, right = get_wheel_coefs(decision)
            # if self._navigator.mid_sector.status != prev_value:  # or (not mid_sec):
            # left, right = 1, 1
            left_velocity = left * const.ROBOT_SPEED
            right_velocity = right * const.ROBOT_SPEED
            # fuzzy_flag = True

            prev_value = self._navigator.mid_sector.status
            # print(prev_value)

            # elif fuzzy_flag and not mid_sec:
            #     fuzzy_flag = False
            #     robot.rotate()
            # else:
            #     # Тут чёткая наводка на цель
            #     print("Чотко")
            #     left_velocity = const.ROBOT_SPEED
            #     right_velocity = const.ROBOT_SPEED

            self._set_movement(left_velocity, right_velocity)

            self.sum_rot += abs(robot.sim.getObjectOrientation(robot.robot_handle, -1)[0] - self.prev_orientation[0])
            self.prev_orientation = robot.sim.getObjectOrientation(robot.robot_handle, -1)

            distance = self._navigator.min_dist
            end = time.time()
            if (not np.isnan(distance)) and (distance < const.ROBOT_STOP_DISTANCE):
                self._set_movement(0, 0)
                break
            target_distance = self.sim.checkDistance(self.robot_handle, self._target_handle, 0)
            if target_distance[1][6] == 0.0:
                self._set_movement(0, 0)
                break
            if end - start > const.CREATURE_LIFETIME:
                self._set_movement(0, 0)
                break

        # print(distance)
        self.sim.setObjectOrientation(self.robot_handle, -1, start_orientation)
        self.sim.setObjectPosition(self.robot_handle, -1, start_position)


if __name__ == "__main__":
    training = True
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    sim.startSimulation()

    robot = Robot(sim, const.ROBOT_NAME, training=training)

    start_orientation = robot.sim.getObjectOrientation(robot.robot_handle, -1)
    start_position = robot.sim.getObjectPosition(robot.robot_handle, -1)

    # Генетический алгоритм:
    if training:
        config = [(0, robot.planner)]
        start_x, start_y, _ = start_position
        target_x, target_y, _ = robot.sim.getObjectPosition(robot._navigator._target_handle,
                                                            robot._navigator._robot_handle)
        start_delta = ((target_x - start_x) ** 2 + (target_y - start_y) ** 2) ** 0.5
        for i in range(const.NUM_CREATURES):
            robot.start()

            # print(robot.target_distance)

            F = robot.planner.survival_function(robot.sum_rot, start_delta,
                                                robot.target_distance)  # Вычисление полезности
            config.append(robot.planner.selection(config[-1], F))  # Добавление в конец списка
            robot.planner = config[-1][1]  # Отбор

            print(F)

            robot.planner.mutate(0.2)  # Мутация
            # robot.planner.crossover(config[i])  # Скрещивание

            robot.sim.setObjectOrientation(robot.robot_handle, -1, start_orientation)
            robot.sim.setObjectPosition(robot.robot_handle, -1, start_position)
    else:
        robot.start()

    sim.stopSimulation()
