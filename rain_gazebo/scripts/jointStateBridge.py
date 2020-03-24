#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from relaxed_ik.msg import JointAngles
from sensor_msgs.msg  import JointState

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('relaxedIKGazebo', anonymous=True)

    #rospy.Subscriber("relaxed_ik/joint_angle_solutions", JointAngles, talker)
    rospy.Subscriber("joint_states", JointState, talker)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker(jAngles):
    pub = rospy.Publisher('joint_group_position_controller/command', Float64MultiArray, queue_size=10)
    #rospy.init_node('relaxedIKGazebo', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
    jointAngles = Float64MultiArray()
    #print(jAngles.angles.data)
    print(jAngles.position)
    #jointAngles.data = jAngles.angles.data
    jointAngles.data = jAngles.position
    jointAngles.data = [jointAngles.data[0], jointAngles.data[1]-1.57,jointAngles.data[2],jointAngles.data[3]-1.57,jointAngles.data[4],jointAngles.data[5]]
    #rospy.loginfo(jointAngles)
    pub.publish(jointAngles)
    #rate.sleep()

if __name__ == '__main__':
    listener()
