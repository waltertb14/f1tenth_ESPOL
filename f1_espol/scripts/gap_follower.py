#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class GapFollower:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 10
    STRAIGHTS_SPEED = 0.5
    CORNERS_SPEED = 0.1
    STRAIGHTS_STEERING_ANGLE = 0.05
    CORNER_THRESHOLD = np.pi / 12  # 코너를 감지하기 위한 임계값
    TURN_ANGLE_MULTIPLIER = 1.5

    def __init__(self):
        rospy.init_node('gap_follower', anonymous=True)
        
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        #self.drive_topic = rospy.get_param('~gap_drive_topic', '/drive')
        self.drive_topic = "vesc/low_level/ackermann_cmd_mux/input/teleop"

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=10)

        self.radians_per_elem = None
        self.is_corner = False

    def preprocess_lidar(self, ranges):
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        proc_ranges = np.array(ranges[90:-90])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]

        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl

        return chosen_slice.start, chosen_slice.stop
    
    def find_best_point(self, start_i, end_i, ranges):
        # Apply a weight to prioritize closer points for sharper turns
        gap_segment = ranges[start_i:end_i]
        weights = np.linspace(1.5, 1, len(gap_segment)) if self.is_corner else np.ones(len(gap_segment))
        weighted_gap = gap_segment * weights
        averaged_max_gap = np.convolve(weighted_gap, np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i
    
    def get_angle(self, range_index, range_len):
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        self.is_corner = abs(lidar_angle) > self.CORNER_THRESHOLD
        steering_angle = lidar_angle / 2 * (self.TURN_ANGLE_MULTIPLIER if self.is_corner else 1)
        return steering_angle

    def scan_callback(self, scan):
        proc_ranges = self.preprocess_lidar(scan.ranges)
        closest = proc_ranges.argmin()

        min_index = int(closest - self.BUBBLE_RADIUS)
        max_index = int(closest + self.BUBBLE_RADIUS)
        if min_index < 0: min_index = 0
        if max_index > len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        steering_angle = self.get_angle(best, len(proc_ranges))
        speed = self.CORNERS_SPEED if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE else self.STRAIGHTS_SPEED
        self.publish_drive(speed, steering_angle)

    def publish_drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        GapFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
