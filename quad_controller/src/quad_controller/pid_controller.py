# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman
import rospy

# iterations: 806 params: [0.44240232939122903, 0.18697918023466067, 0.40378947833278533] best_run: 1283.42210579


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):

        self.target_ = 0.0
        self.max_windup_ = 0.0

        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0

        self.last_error_ = 0.0
        self.error_sum_ = 0.0

        self.last_timestamp_ = 0.0



    def reset(self):
        self.last_error_ = 0.0
        self.last_timestamp_ = 0.0

    def setTarget(self, target):
        self.target_ = target

    def setKP(self, kp):
        self.kp_ = kp

    def setKI(self, ki):
        self.ki_ = ki

    def setKD(self, kd):
        self.kd_ = kd

    def setMaxWindup(self, max_windup):
        self.max_windup_ = max_windup

    def update(self, measured_value, timestamp):
        error = self.target_ - measured_value
        if self.last_timestamp_ < 0.1:
            self.last_timestamp_ = timestamp
            self.last_error_ = error
            return 0.0

        delta_time = timestamp - self.last_timestamp_
        self.last_timestamp_ = timestamp

        p = self.kp_ * error

        self.error_sum_ += error * delta_time
        if self.error_sum_ > self.max_windup_:
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_:
            self.error_sum_ = -self.max_windup_

        i = self.ki_ * self.error_sum_

        d = self.kd_ * (error - self.last_error_) / delta_time

        self.last_error_ = error

        u = p + d + i

        # rospy.loginfo("PID = {:.3f}, {:.3f}, {:.3f} (dt = {:.4f}, tgt = {:.4f})".format(p, i, d, delta_time, self.target_))


        # if u > 1.0:
        #     u = 1.0
        # elif u < -1.0:
        #     u = -1.0

        return u
