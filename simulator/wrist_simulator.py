import rospy
from sensor_msgs.msg import JointState
import numpy as np

q_wrist = [0, 0, 0]

def flexiv_cb2(data):
    global q_wrist 
    q_wrist = data.position[:3]



pub2 = rospy.Publisher('flexiv_wrist_get_js', JointState, queue_size=10)
sub2 = rospy.Subscriber("flexiv_wrist_set_js", JointState, flexiv_cb2)

rospy.init_node('wrist_simulator', anonymous=True)
rate = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    
    msg = JointState()
    msg.position=q_wrist
    pub2.publish(msg)

    rate.sleep()