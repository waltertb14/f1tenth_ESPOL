#!/usr/bin/env python2
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class FollowTheGap:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("follow_the_gap", anonymous=True)
        
        # Publisher: publishes the AckermannDriveStamped message
        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/teleop",
                                   AckermannDriveStamped, queue_size=1)
        
        # Subscriber: subscribe to the laser scan data (from RPLIDAR S2)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        # Parameters (can be set via rosparam)
        self.speed = rospy.get_param("~speed", 0.5)           # Constant speed command
        self.max_steer = rospy.get_param("~max_steer", 0.2)     # Maximum steering angle (radians)
        self.gap_threshold = rospy.get_param("~gap_threshold", 0.5)  # Distance threshold (meters) to consider "free"
        
        rospy.loginfo("FollowTheGap node initialized.")

    def laser_callback(self, scan):
        """
        Callback for processing LaserScan data.
        Implements a basic follow-the-gap algorithm.
        """
        # Convert scan ranges to a numpy array and replace inf with max range value.
        ranges = np.array(scan.ranges)
        ranges = np.where(np.isinf(ranges), scan.range_max, ranges)
        
        # Apply a simple moving average filter (smoothing)
        kernel_size = 5
        kernel = np.ones(kernel_size)/float(kernel_size)
        smooth_ranges = np.convolve(ranges, kernel, mode='same')
        
        # Create a mask for free space (values above the gap threshold)
        free_mask = smooth_ranges > self.gap_threshold
        
        # Get the indices where the space is free
        free_indices = np.where(free_mask)[0]
        if free_indices.size == 0:
            rospy.logwarn("No free gap found, stopping.")
            self.publish_command(0.0, 0.0)
            return
        
        # Find contiguous segments in free_indices.
        # Compute differences between consecutive indices.
        diff = np.diff(free_indices)
        segments = []
        seg_start = free_indices[0]
        for i, d in enumerate(diff):
            if d != 1:
                seg_end = free_indices[i]
                segments.append((seg_start, seg_end))
                seg_start = free_indices[i+1]
        # Add the last segment
        segments.append((seg_start, free_indices[-1]))
        
        # Select the longest gap segment
        longest_seg = max(segments, key=lambda s: s[1] - s[0])
        gap_center_index = int((longest_seg[0] + longest_seg[1]) / 2.0)
        
        # Convert the gap center index to an angle
        # Each index corresponds to an angle: angle = angle_min + index * angle_increment
        gap_center_angle = scan.angle_min + gap_center_index * scan.angle_increment
        
        # Map the gap center angle to a steering command.
        # Clamp the steering command to the range [-max_steer, max_steer]
        steering = np.clip(gap_center_angle, -self.max_steer, self.max_steer)
        
        rospy.loginfo("Gap center angle: %.2f, Steering command: %.2f", gap_center_angle, steering)
        self.publish_command(self.speed, steering)

    def publish_command(self, speed, steering):
        """
        Creates and publishes an AckermannDriveStamped message with the given speed and steering.
        """
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering
        cmd.drive.steering_angle_velocity = 0.0
        cmd.drive.acceleration = 0.0
        cmd.drive.jerk = 0.0
        self.pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = FollowTheGap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
