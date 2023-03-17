import rospy
from geometry_msgs.msg import TwistStamped

def callback(msg: TwistStamped) -> None:
    rospy.loginfo(f"{rospy.get_caller_id()}: I heard {msg}")

def listener() -> None:

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/amiga/vel", TwistStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
