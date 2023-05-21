import math

ROBOT_NAME = 'PioneerP3DX'  #: Название объекта робота
ROBOT_SPEED = 2  #: Скорость робота
ROBOT_STOP_DISTANCE = 0.1  #: Дистанция до препятствия, на которой робот останавливается

SENSORS_UPDATE_PERIOD = 0.1  #: Период обновления данных на сенсоре, с

LIDAR_MAX_DIST = 3  #: Дальность видимости лидара (изменять также в файле сенсора в Коппелии)
LIDAR_NUM_POINTS = 181  #: Количество точек лидара (изменять также в файле сенсора в Коппелии)
LIDAR_MID_SECTOR_WIDTH = 0.66  #: Ширина среднего сектора лидара
LIDAR_DIST_BETWEEN_POINTS = math.pi * LIDAR_MAX_DIST ** 2 /\
                            (LIDAR_NUM_POINTS - 1)  #: Расстояние между двумя точками лидара

TARGET_NAME = 'Goal'

POSITION_EPSILON = 0.001

CREATURE_LIFETIME = 10
NUM_CREATURES = 10
