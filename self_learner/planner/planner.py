# from enum import IntEnum
# from pathlib import Path
#
# import pandas as pd
#
#
# class Planner:
#     class TurningAngle(IntEnum):
#         big_left = 0
#         left = 1
#         small_left = 2
#         small_right = 3
#         right = 4
#         big_right = 5
#
#     def __init__(self):
#         self._decision_dict = {'big left':      self.TurningAngle.big_left,
#                                'left':          self.TurningAngle.left,
#                                'small left':    self.TurningAngle.small_left,
#                                'small right':   self.TurningAngle.small_right,
#                                'right':         self.TurningAngle.right,
#                                'big right':     self.TurningAngle.big_right}
#
#         self._rules = pd.read_csv(f'{Path(__file__).parent}/fuzzy_logic_rules.csv')
#         self._rules.replace({'target_side':     {'right': 0,
#                                                  'left': 1},
#
#                              'left':            {'empty': 0,
#                                                  'far': 1,
#                                                  'medium': 2,
#                                                  'close': 3},
#
#                              'mid':             {'empty': 0,
#                                                  'far': 1,
#                                                  'medium': 2,
#                                                  'close': 3},
#
#                              'right':           {'empty': 0,
#                                                  'far': 1,
#                                                  'medium': 2,
#                                                  'close': 3}},
#                             inplace=True)
#         self._rules['id'] = 64 * self._rules['target_side'] + \
#                             16 * self._rules['left'] + \
#                             4 * self._rules['mid'] + \
#                             self._rules['right']
#         self._rules.set_index('id')
#         self._rules = self._rules['turn']
#
#     def make_decision(self, target_side, left, mid, right):
#         situation = 64 * int(target_side) + 16 * int(left) + 4 * int(mid) + int(right)
#         decision = self._rules[situation]
#         turning_angle = self._decision_dict[decision]
#         return turning_angle

import simpful as sf

RULE1 = "IF (MID IS close) AND (LEFT IS close) THEN (TURN IS TURNRIGHT)"
RULE2 = "IF (MID IS close) AND (RIGHT IS close) THEN (TURN IS TURNLEFT)"
RULE3 = "IF (TARGETSIDE IS right) THEN (TURN IS TURNRIGHT)"
RULE4 = "IF (TARGETSIDE IS left) THEN (TURN IS TURNLEFT)"

# term for close distance
FUZZY_SET1 = sf.FuzzySet(points=[[-1, 1.], [0., 1.], [5, 0]], term="close")
# term for far dist
FUZZY_SET2 = sf.FuzzySet(points=[[0., 0], [5, 1.], [99., 1.]], term="far")
# term for target is in left sec
FUZZY_SET3 = sf.FuzzySet(points=[[-180, 1], [-179, 1.], [1., 0.]], term="left")
# term for target is in RIGHT sec
FUZZY_SET4 = sf.FuzzySet(points=[[-1., 0], [179, 1.], [180., 1.]], term="right")


class Planer:
    def __init__(self):
        self._fuzzy_system = sf.FuzzySystem()

        self._fuzzy_system.add_linguistic_variable("LEFT", sf.LinguisticVariable([FUZZY_SET1, FUZZY_SET2]))
        self._fuzzy_system.add_linguistic_variable("MID", sf.LinguisticVariable([FUZZY_SET1, FUZZY_SET2]))
        self._fuzzy_system.add_linguistic_variable("RIGHT", sf.LinguisticVariable([FUZZY_SET1, FUZZY_SET2]))
        self._fuzzy_system.add_linguistic_variable("TARGETSIDE", sf.LinguisticVariable([FUZZY_SET3, FUZZY_SET4]))

        # Output value.
        self._fuzzy_system.set_crisp_output_value("TURNLEFT", -5)
        self._fuzzy_system.set_crisp_output_value("TURNRIGHT", 5)


def make_solution(leftsec, midsec, rightsec, targside):
    FS = sf.FuzzySystem()
    # Define a linguistic variable.
    # term for close distance
    S_1 = sf.FuzzySet(points=[[-1, 1.], [0., 1.], [5, 0]], term="low")
    # term for high dist
    S_2 = sf.FuzzySet(points=[[0., 0], [5, 1.], [99., 1.]], term="high")
    # term for target is in left sec
    S_3 = sf.FuzzySet(points=[[-180, 1], [-179, 1.], [1., 0.]], term="left")
    # term for target is in RIGHT sec
    S_4 = sf.FuzzySet(points=[[-1., 0], [179, 1.], [180., 1.]], term="right")
    FS.add_linguistic_variable("LEFT", sf.LinguisticVariable([S_1, S_2]))
    FS.add_linguistic_variable("MID", sf.LinguisticVariable([S_1, S_2]))
    FS.add_linguistic_variable("RIGHT", sf.LinguisticVariable([S_1, S_2]))
    FS.add_linguistic_variable("TARGETSIDE", sf.LinguisticVariable([S_3, S_4]))

    # Output value.
    FS.set_crisp_output_value("TURNLEFT", -5)
    FS.set_crisp_output_value("TURNRIGHT", 5)

    # Here are the rules for wall smashing    .
    RULE1 = "IF (MID IS low) AND (LEFT IS low) THEN (TURN IS TURNRIGHT)"
    RULE2 = "IF (MID IS low) AND (RIGHT IS low) THEN (TURN IS TURNLEFT)"
    RULE3 = "IF (TARGETSIDE IS right) THEN (TURN IS TURNRIGHT)"
    RULE4 = "IF (TARGETSIDE IS left) THEN (TURN IS TURNLEFT)"
    FS.add_rules([RULE1, RULE2, RULE3, RULE4])

    # Set antecedents values, perform Sugeno inference and print output values.
    FS.set_variable("LEFT", leftsec)
    FS.set_variable("MID", midsec)
    FS.set_variable("RIGHT", rightsec)
    FS.set_variable("TARGETSIDE", targside)
    return (FS.Sugeno_inference(['TURN']))


print(make_solution(4, 1, 1, -10))
