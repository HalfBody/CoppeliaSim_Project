import numpy as np
from simpful import *


class Fuzzy():
    def __init__(self, LR_arr, LRF_arr):
        self.FS = FuzzySystem()

        LR_n = TrapezoidFuzzySet(a=0, b=0, c=LR_arr[0], d=LR_arr[1], term = 'n')
        LR_m = TrapezoidFuzzySet(a=LR_arr[2], b=LR_arr[3], c=LR_arr[4], d=LR_arr[5], term = 'm')
        LR_f = TrapezoidFuzzySet(a=LR_arr[6], b=LR_arr[7], c=2, d=2, term = 'f')

        LRF_n = TrapezoidFuzzySet(a=0, b=0, c=LRF_arr[0], d=LRF_arr[1], term = 'n')
        LRF_m = TrapezoidFuzzySet(a=LRF_arr[2], b=LRF_arr[3], c=LRF_arr[4], d=LRF_arr[5], term = 'm')
        LRF_f = TrapezoidFuzzySet(a=LRF_arr[6], b=LRF_arr[7], c=2, d=2, term = 'f')

        self.FS.add_linguistic_variable('L', LinguisticVariable([LR_n, LR_m, LR_f], concept="L_dist", universe_of_discourse=[0,2]))
        self.FS.add_linguistic_variable('LF', LinguisticVariable([LRF_n, LRF_m, LRF_f], concept="Lf_dist", universe_of_discourse=[0,2]))
        self.FS.add_linguistic_variable('F', LinguisticVariable([LRF_n, LRF_m, LRF_f], concept="F_dist", universe_of_discourse=[0,2]))
        self.FS.add_linguistic_variable('RF', LinguisticVariable([LRF_n, LRF_m, LRF_f], concept="RF_dist", universe_of_discourse=[0,2]))
        self.FS.add_linguistic_variable('R', LinguisticVariable([LR_n, LR_m, LR_f], concept="R_dist", universe_of_discourse=[0,2]))

        L = TrapezoidFuzzySet(a=-180, b=-180, c=-30, d=-60, term = 'L')
        LF = TrapezoidFuzzySet(a=-70, b=-20, c=-10, d=-5, term = 'LF')
        F = TriangleFuzzySet(a=-10, b=0, c=10, term='F')
        RF = TrapezoidFuzzySet(a=5, b=10, c=20, d=70, term = 'RF')
        R = TrapezoidFuzzySet(a=60, b=30, c=180, d=180, term = 'R')
        self.FS.add_linguistic_variable('target_angle', LinguisticVariable([L, LF, F, RF, R], concept='angle to target', universe_of_discourse=[-180,180]))

        LS = TrapezoidFuzzySet(a=-0.5, b=-0.4, c=-0.3, d=-0.2, term = 'LS')
        LM = TrapezoidFuzzySet(a=-0.3, b=-0.3, c=-0.2, d=-0.1, term = 'LM')
        F = TriangleFuzzySet(a=-0.1, b=0, c=0.1, term='F')
        RM = TrapezoidFuzzySet(a=0.1, b=0.2, c=0.3, d=0.3, term = 'RM')
        RS = TrapezoidFuzzySet(a=0.2, b=0.3, c=0.4, d=0.5, term = 'RS')
        self.FS.add_linguistic_variable('angle', LinguisticVariable([LS, LM, F, RM, RS], concept='turn angle', universe_of_discourse=[-0.5,0.5]))

        # LS = TrapezoidFuzzySet(a=-2, b=-1.5, c=-1, d=-0.5, term = 'LS')
        # LM = TrapezoidFuzzySet(a=-1.5, b=-1, c=-0.5, d=-0, term = 'LM')
        # F = TriangleFuzzySet(a=-0.5, b=0, c=0.5, term='F')
        # RM = TrapezoidFuzzySet(a=0, b=0.5, c=1, d=1.5, term = 'RM')
        # RS = TrapezoidFuzzySet(a=0.5, b=1, c=1.5, d=2, term = 'RS')
        # self.FS.add_linguistic_variable('angle', LinguisticVariable([LS, LM, F, RM, RS], concept='turn angle', universe_of_discourse=[-90,90]))

        reverse = TriangleFuzzySet(a = -2, b = -1.5, c = 0, term = 'reverse')
        slow = TriangleFuzzySet(a=1, b=1.5, c=2, term='slow')
        normal = TriangleFuzzySet(a=2.5, b=3, c=3.5, term='normal')
        self.FS.add_linguistic_variable('speed', LinguisticVariable([reverse, slow, normal], concept='speed', universe_of_discourse=[-1,3]))

        rules = []
        rules.append('IF (L IS n) THEN (angle IS RM)')
        rules.append('IF (LF IS n) THEN (angle IS RM)')
        rules.append('IF (RF IS n) THEN (angle IS LM)')
        rules.append('IF (R IS n) THEN (angle IS LM)')
        rules.append('IF (L IS n) AND (LF IS n) AND (F IS n) THEN (angle IS RS)')
        rules.append('IF (LF IS n) AND (F IS n) THEN (angle IS RS)')
        rules.append('IF (R IS n) AND (RF IS n) AND (F IS n) THEN (angle IS LS)')
        rules.append('IF (RF IS n) AND (F IS n) THEN (angle IS LS)')
        rules.append('IF (L IS n) AND (LF IS n) THEN (angle IS RS)')
        rules.append('IF (R IS n) AND (RF IS n) THEN (angle IS LS)')

        rules.append('IF (L IS m) THEN (angle IS F)')
        rules.append('IF (LF IS m) THEN (angle IS RM)')
        rules.append('IF (RF IS m) THEN (angle IS LM)')
        rules.append('IF (R IS m) THEN (angle IS F)')
        rules.append('IF (L IS m) AND (LF IS m) AND (F IS m) THEN (angle IS RM)')
        rules.append('IF (LF IS m) AND (F IS m) THEN (angle IS RS)')
        rules.append('IF (R IS m) AND (RF IS m) AND (F IS m) THEN (angle IS LM)')
        rules.append('IF (RF IS m) AND (F IS m) THEN (angle IS LM)')
        rules.append('IF (L IS m) AND (LF IS m) THEN (angle IS RM)')
        rules.append('IF (R IS m) AND (RF IS m) THEN (angle IS LM)')

        # rules.append('IF (target_angle IS L) THEN (angle IS LM)')
        # rules.append('IF (target_angle IS LF) THEN (angle IS LM)')
        # rules.append('IF (target_angle IS R) THEN (angle IS RM)')
        # rules.append('IF (target_angle IS RF) THEN (angle IS RM)')
        # rules.append('IF (target_angle IS F) THEN (angle IS F)')

        # rules.append('IF (LF IS n) AND (RF IS n) AND (target_angle IS LF) THEN (angle IS LM)')
        # rules.append('IF (LF IS n) AND (RF IS n) AND (target_angle IS RF) THEN (angle IS RM)')
        # rules.append('IF (LF IS n) AND (RF IS n) AND (target_angle IS L) THEN (angle IS LS)')
        # rules.append('IF (LF IS n) AND (RF IS n) AND (target_angle IS R) THEN (angle IS RS)')
        # rules.append('IF (LF IS n) AND (L IS n) AND (target_angle IS L) THEN (angle IS LS)')
        # rules.append('IF (RF IS n) AND (R IS n) AND (target_angle IS R) THEN (angle IS RS)')
        # rules.append('IF (LF IS n) AND (RF IS n) AND (target_angle IS F) THEN (angle IS LS)')

        rules.append('IF (LF IS m) THEN (speed IS noraml)')
        rules.append('IF (RF IS m) THEN (speed IS normal)')
        rules.append('IF (L IS m) AND (LF IS m) AND (RF IS m) THEN (speed IS slow)')
        rules.append('IF (R IS m) AND (LF IS m) AND (RF IS m) THEN (speed IS slow)')
        rules.append('IF (LF IS m) AND (F IS m) THEN (speed IS slow)')
        rules.append('IF (RF IS m) AND (F IS m) THEN (speed IS slow)')
        rules.append('IF (F IS m) THEN (speed IS slow)')

        rules.append('IF (LF IS n) THEN (speed IS slow)')
        rules.append('IF (RF IS n) THEN (speed IS slow)')
        rules.append('IF (L IS n) AND (LF IS n) AND (RF IS n) THEN (speed IS slow)')
        rules.append('IF (R IS n) AND (LF IS n) AND (RF IS n) THEN (speed IS slow)')
        rules.append('IF (LF IS n) AND (F IS n) THEN (speed IS reverse)')
        rules.append('IF (RF IS n) AND (F IS n) THEN (speed IS reverse)')
        rules.append('IF (F IS n) THEN (speed IS reverse)')

        rules.append('IF (target_angle IS L) THEN (speed IS slow)')
        rules.append('IF (target_angle IS LF) THEN (speed IS normal)')
        rules.append('IF (target_angle IS R) THEN (speed IS slow)')
        rules.append('IF (target_angle IS RF) THEN (speed IS normal)')
        rules.append('IF (target_angle IS F) THEN (speed IS normal)')

        self.FS.add_rules(rules)


    def get_lidar_data(self, lidar_data):
        lidar = np.array_split(lidar_data, 5)
        var_data = [min(dist) for dist in lidar]
                    
        return var_data
        
        
    def fuz_log(self, lidar_data, target_angle):
        var_data = self.get_lidar_data(lidar_data)

        self.FS.set_variable("L", var_data[0])
        self.FS.set_variable("LF", var_data[1])
        self.FS.set_variable("F", var_data[2])
        self.FS.set_variable("RF", var_data[3])
        self.FS.set_variable("R", var_data[4])
        self.FS.set_variable("target_angle", target_angle)

        return self.FS.Mamdani_inference(['angle', 'speed'])