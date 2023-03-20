import pandas as pd
from enum import IntEnum


class Planner:
    class TurningAngle(IntEnum):
        big_left = 0
        left = 1
        small_left = 2
        small_right = 3
        right = 4
        big_right = 5

    def __init__(self):
        self._decision_dict = {'big left':      self.TurningAngle.big_left,
                               'left':          self.TurningAngle.left,
                               'small left':    self.TurningAngle.small_left,
                               'small right':   self.TurningAngle.small_right,
                               'right':         self.TurningAngle.right,
                               'big right':     self.TurningAngle.big_right}

        self._rules = pd.read_csv('fuzzy_logic_rules.csv')
        self._rules.replace({'target_side':     {'right': 0,
                                                 'left': 1},

                             'left':            {'empty': 0,
                                                 'far': 1,
                                                 'medium': 2,
                                                 'close': 3},

                             'mid':             {'empty': 0,
                                                 'far': 1,
                                                 'medium': 2,
                                                 'close': 3},

                             'right':           {'empty': 0,
                                                 'far': 1,
                                                 'medium': 2,
                                                 'close': 3}},
                            inplace=True)
        self._rules['id'] = 64 * self._rules['target_side'] + \
                            16 * self._rules['left'] + \
                            4 * self._rules['mid'] + \
                            self._rules['right']
        self._rules.set_index('id')
        self._rules = self._rules['turn']

    def make_decision(self, target_side, left, mid, right):
        situation = 64 * int(target_side) + 16 * int(left) + 4 * int(mid) + int(right)
        decision = self._rules[situation]
        turning_angle = self._decision_dict[decision]
        return turning_angle
