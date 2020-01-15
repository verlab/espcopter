#! /usr/bin/env python
import rospy
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Joy

def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

def callback(data):
    global pub, trim_x, trim_y, trim_z
    cmd_ = AttitudeTarget()
    cmd_.body_rate.x = valmap(-data.axes[0], -1, 1, -100, 100) + trim_x
    cmd_.body_rate.y = valmap(data.axes[1], -1, 1,  -100, 100) + trim_y
    cmd_.body_rate.z = valmap(-data.axes[2], -1, 1,  -200, 200) + trim_z
    #cmd_.body_rate.x = valmap(-data.axes[0], -1, 1, -1250, 1250)
    #cmd_.body_rate.y = valmap(data.axes[1], -1, 1, -1250, 1250)
    #cmd_.body_rate.z = valmap(data.axes[2], -1, 1, -6000, 6000)
    cmd_.thrust = valmap(data.axes[3], -1, 1, 0, 700)
    if data.buttons[11]:
        trim_x += 1
        rospy.loginfo("trim_x: " + str(trim_x))
    if data.buttons[10]:
        trim_x -= 1
        rospy.loginfo("trim_x: " + str(trim_x))
    if data.buttons[9]:
        trim_y += 1
        rospy.loginfo("trim_y: " + str(trim_y))
    if data.buttons[8]:
        trim_y -= 1
        rospy.loginfo("trim_y: " + str(trim_y))
    if data.buttons[7]:
        trim_z += 1
        rospy.loginfo("trim_z: " + str(trim_z))
    if data.buttons[6]:
        trim_z -= 1
        rospy.loginfo("trim_z: " + str(trim_z))
    
    pub.publish(cmd_)

# Intializes everything
def start():
    global pub, trim_x, trim_y, trim_z
    trim_x = 0
    trim_y = 0
    trim_z = 0

    pub = rospy.Publisher('drone_1/attitude', AttitudeTarget, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('espcopter_teleop')
    rospy.loginfo("Starting")
    rospy.spin()


if __name__ == '__main__':
    start()