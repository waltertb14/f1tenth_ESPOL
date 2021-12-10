#!/usr/bin/env python2
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from pynput.keyboard import Key, Listener
import threading

# Valores según tu mapeo
SPEED_UP      = 2.0    # cuando se presiona flecha arriba
SPEED_DOWN    = -2.0   # cuando se presiona flecha abajo
STEER_LEFT    = 0.27   # cuando se presiona flecha izquierda
STEER_RIGHT   = -0.27  # cuando se presiona flecha derecha
STEER_NEUTRAL = 0.0    # cuando no se presiona ninguna flecha lateral

# Variables globales para mantener el último comando
current_speed = 0.0
current_steering = STEER_NEUTRAL

pub = None

def publish_loop():
    rate = rospy.Rate(10)  # Publicar a 10 Hz
    global current_speed, current_steering
    while not rospy.is_shutdown():
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = current_speed
        cmd.drive.steering_angle = current_steering
        cmd.drive.steering_angle_velocity = 0.0
        cmd.drive.acceleration = 0.0
        cmd.drive.jerk = 0.0
        pub.publish(cmd)
        rate.sleep()

def on_press(key):
    global current_speed, current_steering
    try:
        if key == Key.up:
            current_speed = SPEED_UP
            current_steering = STEER_NEUTRAL
            rospy.loginfo("Up pressed: speed=%.2f", SPEED_UP)
        elif key == Key.down:
            current_speed = SPEED_DOWN
            current_steering = STEER_NEUTRAL
            rospy.loginfo("Down pressed: speed=%.2f", SPEED_DOWN)
        elif key == Key.left:
            current_speed = 0.0
            current_steering = STEER_LEFT
            rospy.loginfo("Left pressed: steering=%.2f", STEER_LEFT)
        elif key == Key.right:
            current_speed = 0.0
            current_steering = STEER_RIGHT
            rospy.loginfo("Right pressed: steering=%.2f", STEER_RIGHT)
        elif key == Key.esc:
            # Termina el listener
            return False
    except Exception as e:
        rospy.logerr("Error in on_press: %s", str(e))

def on_release(key):
    global current_speed, current_steering
    # Al soltar cualquier tecla, restablecemos a estado neutro
    current_speed = 0.0
    current_steering = STEER_NEUTRAL
    rospy.loginfo("Key released: resetting to neutral")

def main():
    global pub
    rospy.init_node('keyboard_ackermann_control', anonymous=True)
    pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',
                          AckermannDriveStamped, queue_size=1)

    # Iniciar hilo para publicar continuamente
    pub_thread = threading.Thread(target=publish_loop)
    pub_thread.daemon = True
    pub_thread.start()

    rospy.loginfo("Nodo keyboard_ackermann_control iniciado. Presiona Esc para salir.")
    # Inicia el listener del teclado (se recomienda ejecutarlo en el entorno gráfico local)
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

