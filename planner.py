import pandas as pd


class Planner:
    def __init__(self):
        self._rules = pd.read_csv('fuzzy_logic_rules.csv')
        self._rules.replace({'target_side': {'right': 0,
                                             'left': 1},

                             'left':        {'empty': 0,
                                             'far': 1,
                                             'medium': 2,
                                             'close': 3},

                             'mid':         {'empty': 0,
                                             'far': 1,
                                             'medium': 2,
                                             'close': 3},

                             'right':       {'empty': 0,
                                             'far': 1,
                                             'medium': 2,
                                             'close': 3}},
                            inplace=True)
        self._rules['id'] = 64 * self._rules['target_side'] + \
                            16 * self._rules['left'] + \
                            4 * self._rules['mid'] + \
                            self._rules['right']
        self._rules.set_index('id')

    def get_decision(self, target_side, left, mid, right):
        ...

