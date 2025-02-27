#!/usr/bin/env python2
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from pynput.keyboard import Key, Listener

# Global variable for the current speed (starts at 0.0)
current_speed = 0.0

# Default steering angle (neutral)
STEER_NEUTRAL = 0.0

# Global publisher (will be initialized in main)
pub = None

def publish_command():
    """
    Creates and publishes an AckermannDriveStamped message with the current speed
    and a neutral steering angle. This function is called continuously in the main loop.
    """
    global current_speed, pub
    cmd = AckermannDriveStamped()
    cmd.header.stamp = rospy.Time.now()
    # Set the speed and steering angle in the message
    cmd.drive.speed = current_speed
    cmd.drive.steering_angle = STEER_NEUTRAL
    # Set other fields to zero (can be adjusted if needed)
    cmd.drive.steering_angle_velocity = 0.0
    cmd.drive.acceleration = 0.0
    cmd.drive.jerk = 0.0

    pub.publish(cmd)
    rospy.loginfo("Published command: speed = %.2f, steering = %.2f", current_speed, STEER_NEUTRAL)

def on_press(key):
    """
    Callback function that is triggered on key press.
    Increases speed by 0.1 when 'q' is pressed,
    decreases speed by 0.1 when 'w' is pressed.
    """
    global current_speed
    try:
        # Check if the key has a 'char' attribute (to filter out special keys)
        if hasattr(key, 'char'):
            if key.char == 'q':
                current_speed += 0.25
                rospy.loginfo("Increasing speed to: %.2f", current_speed)
            elif key.char == 'w':
                current_speed -= 0.25
                rospy.loginfo("Decreasing speed to: %.2f", current_speed)
    except Exception as e:
        rospy.logerr("Error in on_press: %s", str(e))

def on_release(key):
    """
    Callback function for key release.
    Here, if the Esc key is released, the keyboard listener will stop.
    """
    if key == Key.esc:
        # Stop listener by returning False
        return False

def main():
    global pub
    # Initialize the ROS node
    rospy.init_node('keyboard_speed_control', anonymous=True)
    
    # Create a publisher for the teleop topic
    pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',
                          AckermannDriveStamped, queue_size=1)
    
    # Set a publishing rate (e.g., 10 Hz)
    rate = rospy.Rate(10)
    
    # Start the keyboard listener in a separate thread
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    rospy.loginfo("Keyboard speed control node started. Press 'q' to increase speed, 'w' to decrease speed, and Esc to exit.")

    # Main loop: continuously publish the command message
    while not rospy.is_shutdown():
        publish_command()
        rate.sleep()
    
    # Wait for the listener thread to finish (if necessary)
    listener.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
