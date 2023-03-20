from dataclasses import dataclass
from enum import IntEnum
from threading import Thread
from time import sleep

import numpy as np
from numpy import NaN

import constants as const
from coppeliasim_api.zmqRemoteApi import RemoteAPIClient


class TargetSide(IntEnum):
    right = 0
    left = 1


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

    def __init__(self):
        self._points: list[SensorPoint] = []
        self._status = self.Status.empty
        self._min_dist = NaN

    @property
    def status(self):
        return self._status

    @property
    def min_dist(self):
        return self._min_dist

    @property
    def points(self):
        return self._points

    @points.setter
    def points(self, points):
        """Добавление точек в каждый сектор сенсора.
        Определение минимальной дистанции до препятствия, если оно есть,
        и определение значения терма
        """
        self._points = points
        min_dist = np.nanmin([p.dist for p in self._points])
        # TODO: Заменить на дефаззификацию(?)
        if min_dist == NaN:
            self._status = self.Status.empty
        elif 0 <= min_dist <= 2:
            self._status = self.Status.close
        elif 1 < min_dist <= 4:
            self._status = self.Status.medium
        elif 4 < min_dist <= 5:
            self._status = self.Status.far

        self._min_dist = min_dist


class Map(list):
    def __init__(self):
        super().__init__()
        self._points: list[float, float] = []

    @property
    def points(self):
        return self._points

    def append(self, point):
        self._points.append(point)
        return self.points


class Navigator:
    def __init__(self):
        def mid_sector_angles():
            """
            Расчёт граничных углов среднего сектора для определения граничных
            точек на максимальной дистанции видимости лидара

            Returns
            -------
            left : float
                Левый угол
            right : float
                Правый угол
            """
            left = np.arccos((const.LIDAR_MID_SECTOR_WIDTH / 2) / const.LIDAR_MAX_DIST)
            right = np.pi - left
            return left, right

        client = RemoteAPIClient()
        self._sim = client.getObject('sim')
        self._robot_handle = self._sim.getObject('/' + const.ROBOT_NAME)
        self._target_handle = self._sim.getObject('/' + const.TARGET_NAME)

        #: Инициализация стороны цели
        self._target_side = self._get_target_position()

        #: Инициализация секторов сенсора
        self._left_sector = SensorSector()
        self._mid_sector = SensorSector()
        self._right_sector = SensorSector()
        self._mid_sector_left_angle, self._mid_sector_right_angle = mid_sector_angles()
        self.min_dist = NaN

        #: Инициализация карты
        self._map = Map()

        #: Отдельный тред для постоянного получения данных о расположении цели и наличии препятствий
        self._sensor_controller = Thread(target=self._monitor_sensors,
                                         daemon=True,
                                         name="Sensor Monitor")
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

    @property
    def target_side(self):
        return self._target_side

    def obstacle_between_robot_and_target(self, robot_handle, target_handle):
        x1, y1 = self._sim.getPosition(robot_handle)[:1]  #: Координаты робота
        x2, y2 = self._sim.getPosition(target_handle)[:1]  #: Координаты цели
        for point in self._map:
            x0, y0 = point[0], point[1]  #: Координаты точки на карте

            #: Поиск препятствия в рамках прямоугольника,
            # диагональю которого является отрезок с вершинами — местами расположения робота и цели
            if min(x1, x2) <= x0 <= max(x1, x2) and \
                    min(y1, y2) <= y0 <= max(y1, y2):
                try:
                    dist = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / \
                           np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)  #: Расстояние от точки до отрезка робот-цель
                    #: Если расстояние от какой-либо точки, присутствующей на карте,
                    # до отрезка робот-цель меньше половины ширины среднего сектора,
                    # то считаем, что между роботом и целью есть препятствие.
                    # Другой вариант: меньше половины расстояния между точками лидара.
                    # if dist <= const.LIDAR_DIST_BETWEEN_POINTS / 2:
                    if dist < const.LIDAR_MID_SECTOR_WIDTH / 2:
                        return True
                except ZeroDivisionError:
                    print("Цель уже достигнута!")
                return False

    def _get_target_position(self):
        target_coords = self._sim.getObjectPosition(self._target_handle, self._robot_handle)
        if target_coords[1] > 0:
            return TargetSide.left
        else:
            return TargetSide.right

    def _monitor_sensors(self):
        """Получение данных с датчиков.
        Заполнение секторов лидара точками.
        Определение стороны, с которой находится цель.
        Данные обновляются раз в UPDATE_RATE секунд в фоновом режиме
        """
        while True:
            left_sector_points = []
            mid_sector_points = []
            right_sector_points = []

            result = bool(self._sim.getInt32Signal("r"))

            if result:  #: Есть ли препятствия в области видимости лидара
                point_abs_signal = self._sim.getStringSignal("pointDataAbs")
                points = self._sim.unpackTable(point_abs_signal)
                absolute_coords_2d = [[pt[0], pt[1]] if pt != {} else [NaN, NaN] for pt in points]

                point_rel_signal = self._sim.getStringSignal("pointDataRel")
                points = self._sim.unpackTable(point_rel_signal)
                relative_coords_2d = [pt if pt != [0, 0] else [NaN, NaN] for pt in points]

                dist_signal = self._sim.getStringSignal("distData")
                dists = self._sim.unpackTable(dist_signal)
                dists = [d if d != 0 else NaN for d in dists]
            else:
                absolute_coords_2d = [[NaN, NaN]] * const.LIDAR_NUM_POINTS
                relative_coords_2d = [[NaN, NaN]] * const.LIDAR_NUM_POINTS
                dists = [NaN] * const.LIDAR_NUM_POINTS

            for i in range(const.LIDAR_NUM_POINTS):
                sector_point = SensorPoint(absolute_coords_2d[i][0],
                                           absolute_coords_2d[i][1],
                                           relative_coords_2d[i][0],
                                           relative_coords_2d[i][1],
                                           dists[i],
                                           i * np.pi / 180)

                #: Добавление точки на карту
                point_2d = [sector_point.x_abs, sector_point.y_abs]
                if point_2d not in self._map:
                    self._map.append(point_2d)

                #: Определение принадлежности точки к секторам
                if not np.isnan(sector_point.x_rel):
                    if sector_point.x_rel < -const.LIDAR_MID_SECTOR_WIDTH / 2:
                        left_sector_points.append(sector_point)
                    elif -const.LIDAR_MID_SECTOR_WIDTH / 2 <= sector_point.x_rel <= const.LIDAR_MID_SECTOR_WIDTH / 2:
                        mid_sector_points.append(sector_point)
                    else:
                        right_sector_points.append(sector_point)
                else:
                    if sector_point.angle < self._mid_sector_left_angle:
                        left_sector_points.append(sector_point)
                    elif self._mid_sector_left_angle <= sector_point.angle <= self._mid_sector_right_angle:
                        mid_sector_points.append(sector_point)
                    else:
                        right_sector_points.append(sector_point)

            self._left_sector.points = left_sector_points
            self._mid_sector.points = mid_sector_points
            self._right_sector.points = right_sector_points
            self.min_dist = np.nanmin(
                [self._left_sector.min_dist, self._mid_sector.min_dist, self._right_sector.min_dist])

            self._target_side = self._get_target_position()

            sleep(const.SENSORS_UPDATE_PERIOD)
