from dataclasses import dataclass
from enum import IntEnum
from threading import Thread
from time import sleep

import numpy as np
from numpy import NaN

from constants import *
from zmqRemoteApi import RemoteAPIClient


@dataclass
class SensorPoint:
    x_abs: float
    y_abs: float
    x_rel: float
    y_rel: float
    dist: float
    angle: float


class SensorSector:
    class Status(IntEnum):
        empty = 0
        far = 1
        medium = 2
        close = 3

    points: list[SensorPoint] = []

    def __init__(self):
        self._status = self.Status.empty
        self.min_dist = NaN

    @property
    def status(self):
        return self._status

    def _set_points(self, points):
        """Добавление точек в каждый сектор сенсора.
        Определение минимальной дистанции до препятствия, если оно есть,
        и определение значения терма
        """
        self.points = points
        min_dist = np.nanmin([p.dist for p in self.points])
        # TODO: Заменить на дефаззификацию(?)
        if min_dist == NaN:
            self._status = self.Status.empty
        elif 0 <= min_dist <= 2:
            self._status = self.Status.close
        elif 1 < min_dist <= 4:
            self._status = self.Status.medium
        elif 4 < min_dist <= 5:
            self._status = self.Status.far

        self.min_dist = min_dist


class Sensor:
    def __init__(self):
        def mid_sector_angles():
            left = np.arccos((SENSOR_MID_SECTOR_WIDTH / 2) / SENSOR_MAX_DIST)
            right = np.pi - left
            return left, right

        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

        self._left_sector = SensorSector()
        self._mid_sector = SensorSector()
        self._right_sector = SensorSector()
        self._mid_sector_left_angle, self._mid_sector_right_angle = mid_sector_angles()
        self.min_dist = NaN

        self._sensor_controller = Thread(target=self._monitor_sensor, daemon=True, name="Sensor Monitor")
        self._sensor_controller.start()

    @property
    def left_sector(self):
        return self._left_sector

    @property
    def mid_sector(self):
        return self._mid_sector

    @property
    def right_sector(self):
        return self._right_sector

    def _monitor_sensor(self):
        """Получение данных с датчика.
        Заполнение секторов сенсора точками.
        Данные обновляются раз в UPDATE_RATE секунд в фоновом режиме
        """
        while True:
            left_sector_points = []
            mid_sector_points = []
            right_sector_points = []

            result = bool(self.sim.getInt32Signal("r"))

            if result:
                point_abs_signal = self.sim.getStringSignal("pointDataAbs")
                points = self.sim.unpackTable(point_abs_signal)
                absolute_coords_2d = [[pt[0], pt[1]] if pt != {} else [NaN, NaN] for pt in points]

                point_rel_signal = self.sim.getStringSignal("pointDataRel")
                points = self.sim.unpackTable(point_rel_signal)
                relative_coords_2d = [pt if pt != [0, 0] else [NaN, NaN] for pt in points]

                dist_signal = self.sim.getStringSignal("distData")
                dists = self.sim.unpackTable(dist_signal)
                dists = [d if d != 0 else NaN for d in dists]
            else:
                absolute_coords_2d = [[NaN, NaN]] * SENSOR_NUM_POINTS
                relative_coords_2d = [[NaN, NaN]] * SENSOR_NUM_POINTS
                dists = [NaN] * SENSOR_NUM_POINTS

            for i in range(SENSOR_NUM_POINTS):
                point = SensorPoint(absolute_coords_2d[i][0],
                                    absolute_coords_2d[i][1],
                                    relative_coords_2d[i][0],
                                    relative_coords_2d[i][1],
                                    dists[i],
                                    i * np.pi / 180)
                if not np.isnan(point.x_rel):
                    if point.x_rel < -SENSOR_MID_SECTOR_WIDTH / 2:
                        left_sector_points.append(point)
                    elif -SENSOR_MID_SECTOR_WIDTH / 2 <= point.x_rel <= SENSOR_MID_SECTOR_WIDTH / 2:
                        mid_sector_points.append(point)
                    else:
                        right_sector_points.append(point)
                else:
                    if point.angle < self._mid_sector_left_angle:
                        left_sector_points.append(point)
                    elif self._mid_sector_left_angle <= point.angle <= self._mid_sector_right_angle:
                        mid_sector_points.append(point)
                    else:
                        right_sector_points.append(point)

            self._left_sector._set_points(left_sector_points)
            self._mid_sector._set_points(mid_sector_points)
            self._right_sector._set_points(right_sector_points)
            self.min_dist = np.nanmin(
                [self._left_sector.min_dist, self._mid_sector.min_dist, self._right_sector.min_dist])

            sleep(SENSOR_UPDATE_PERIOD)


class Navigator(Sensor):
    class TargetSide(IntEnum):
        right = 0
        left = 1

    def __init__(self):
        def get_target_side():
            robot_handle = self.sim.getObject('/' + ROBOT_NAME)
            target_handle = self.sim.getObject('/' + TARGET_NAME)
            coords = self.sim.getObjectPosition(target_handle, robot_handle)
            if coords[0] < 0:
                return self.TargetSide.left
            else:
                return self.TargetSide.right

        super().__init__()

        self.target_side = get_target_side()
