#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from relaxed_ik.msg import JointAngles
from relaxed_ik.msg import EEPoseGoals
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from sensor_msgs.msg  import JointState
from visualization_msgs.msg import Marker

relaxedIK = get_relaxedIK_from_info_file("/home/neil/catkin_ws/src/relaxed_ik/src")
init_ee_positions =  relaxedIK.vars.init_ee_positions
init_ee_quats =  relaxedIK.vars.init_ee_quats

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('relaxedIKGazebo', anonymous=True)

    rospy.Subscriber("relaxed_ik/joint_angle_solutions", JointAngles, talker)
    
    rospy.Subscriber("relaxed_ik/ee_pose_goals", EEPoseGoals, markerTalker)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker(jAngles):
    pub = rospy.Publisher('joint_group_position_controller/command', Float64MultiArray, queue_size=10)

    jointAngles = Float64MultiArray()
    #print(jAngles.angles.data)
    
    jointAngles.data = jAngles.angles.data
    
    jointAngles.data = [jointAngles.data[0], jointAngles.data[1]-1.57,jointAngles.data[2],jointAngles.data[3]-1.57,jointAngles.data[4],jointAngles.data[5]]
    #rospy.loginfo(jointAngles)
    pub.publish(jointAngles)
    
def markerTalker(mPose):
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker = Marker()
    marker.pose.position.x = mPose.ee_poses[0].position.x + init_ee_positions[0][0]
    marker.pose.position.y = mPose.ee_poses[0].position.y + init_ee_positions[0][1]
    marker.pose.position.z = mPose.ee_poses[0].position.z + init_ee_positions[0][2]

    marker.pose.orientation = mPose.ee_poses[0].orientation
    marker.header.frame_id = "common_world"
    marker.header.stamp = rospy.Time.now()
    marker.scale.x = 0.06
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1
    marker.color.r = 1

    #print(mPose.ee_poses[0].position.x - init_ee_positions[0][0])
    pub.publish(marker)

if __name__ == '__main__':
    listener()
