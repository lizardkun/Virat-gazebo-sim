#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# simple 'wasd' teleop code that works like video game controls
# everything works except 's' also moves forward for some reason, i think it may be a physics error

rospy.init_node("virat_teleop")
pub = rospy.Publisher("virat/cmd_vel", Twist, queue_size=3)

msg = """
VIRAT CONTROLLER
|---------------|
|PRESS:         |
|W-forward      |
|A-turns left   |
|S-backward     |
|D-turns right  |
|---------------|
ctrl-C to end programme
"""
# asigning speed in form (linear,angular)
CmdKeys = {"w": (20, 0), "a": (0, -20), "s": (-20, 0), "d": (0, 20)}
lin_x = 0
ang_z = 0

# getkey function taken from https://github.com/devanshdhrafani/diff_drive_bot/blob/master/scripts/keyboard_teleop.py
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        input_key = sys.stdin.read(1)
    else:
        input_key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return input_key


settings = termios.tcgetattr(sys.stdin)


# error handling
try:
    print(msg)

    while 1:
        input_key = getKey()
        # taking wasd inputs from user and assigning velocity accordingly
        if input_key in CmdKeys.keys():
            lin_x = CmdKeys[input_key][0]
            ang_z = CmdKeys[input_key][1]
        elif input_key == "":
            lin_x = 0
            ang_z = 0
        # ctrl-C to escape
        elif input_key == "\x03":
            break

        twist = Twist()
        twist.linear.x = lin_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = ang_z

        # publishing velocity
        pub.publish(twist)

except Exception as e:
    print(e)

finally:
    twist = Twist()
    twist.linear.x = lin_x
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = ang_z

    pub.publish(twist)


termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
