import rospy
from std_msgs.msg import String
import time

def callback1(msg):
    rospy.loginfo(f"Callback1 received: {msg.data}")
    time.sleep(0.1)  

def callback2(msg):
    rospy.loginfo(f"Callback2 received: {msg.data}")

if __name__ == "__main__":
    rospy.init_node("multi_threaded_node")

    rospy.Subscriber("chatter", String, callback1, queue_size=100)
    rospy.Subscriber("chatter_header", String, callback2, queue_size=100)

    spinner = rospy.AsyncSpinner(2) # 2 hilos
    spinner.start()

    rospy.spin()
