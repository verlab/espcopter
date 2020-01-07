#! /usr/bin/env python
import rospy
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Joy

def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

def callback(data):
    cmd_ = AttitudeTarget()
    cmd_.body_rate.x = valmap(-data.axes[0], -1, 1, -500, 500) + 0
    cmd_.body_rate.y = valmap(data.axes[1], -1, 1,  -500, 500) + 0
    cmd_.body_rate.z = valmap(-data.axes[2], -1, 1,  -600, 600) + 0
    #cmd_.body_rate.x = valmap(-data.axes[0], -1, 1, -1250, 1250)
    #cmd_.body_rate.y = valmap(data.axes[1], -1, 1, -1250, 1250)
    #cmd_.body_rate.z = valmap(data.axes[2], -1, 1, -6000, 6000)
    cmd_.thrust = valmap(data.axes[3], -1, 1, 0, 700)
    pub.publish(cmd_)

# Intializes everything
def start():
    global pub

    pub = rospy.Publisher('drone_1/attitude', AttitudeTarget, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('espcopter_teleop')
    rospy.loginfo("Starting")
    rospy.spin()


if __name__ == '__main__':
    start()