import rospy
from sensor_msgs.msg import JointState

q = [0, 0, 0, 0, 0, 0, 0]
q_wrist = [0, 0, 0]

def flexiv_cb(data):
    global q 
    q = data.position

def flexiv_cb(data):
    global q_wrist 
    q_wrist = data.position


pub = rospy.Publisher('flexiv_get_js', JointState, queue_size=10)
sub = rospy.Subscriber("flexiv_set_js", JointState, flexiv_cb)

pub2 = rospy.Publisher('flexiv_wrist_get_js', JointState, queue_size=10)
sub2 = rospy.Subscriber("flexiv_wrist_set_js", JointState, flexiv_cb)

rospy.init_node('flexiv_simulator', anonymous=True)
rate = rospy.Rate(100) # 10hz
while not rospy.is_shutdown():
    msg = JointState()
    msg.position=q

    pub.publish(msg)
    

    msg = JointState()
    msg.position=q_wrist
    pub2.publish(msg)

    rate.sleep()