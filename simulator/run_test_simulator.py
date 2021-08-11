import rospy
from sensor_msgs.msg import JointState
import time

set_q = [1, 2, 3, 4, 5, 0, 0]

def flexiv_cb(data):
    q = data.position
    print(f'get q: {q}')

pub = rospy.Publisher('flexiv_set_js', JointState, queue_size=10)
sub = rospy.Subscriber("flexiv_get_js", JointState, flexiv_cb)
rospy.init_node('flexiv_test', anonymous=True)
msg = JointState()
msg.position=set_q

pub.publish(msg)
time.sleep(0.1)
