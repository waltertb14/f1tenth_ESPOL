#!/usr/bin/env python2
import rospy
import sys
from ackermann_msgs.msg import AckermannDriveStamped

def main():
    if len(sys.argv) < 3:
        print("Uso: {} <speed> <steering_angle>".format(sys.argv[0]))
        sys.exit(1)
    
    try:
        speed = float(sys.argv[1])
        steering_angle = float(sys.argv[2])
    except ValueError:
        print("Los parámetros deben ser números (float).")
        sys.exit(1)

    rospy.init_node("override_ackermann_command", anonymous=True)
    pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/teleop",
                          AckermannDriveStamped, queue_size=1)
    
    rate = rospy.Rate(10)  # Publica a 10 Hz

    rospy.loginfo("Enviando comando continuo: speed=%.2f, steering_angle=%.2f", speed, steering_angle)
    while not rospy.is_shutdown():
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = speed
        cmd.drive.steering_angle = steering_angle
        cmd.drive.steering_angle_velocity = 0.0
        cmd.drive.acceleration = 0.0
        cmd.drive.jerk = 0.0
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
