import rospy
from geometry_msgs.msg import Twist


if __name__== "__main__":
    # define the actions the publisher will make
    pub = rospy.Publisher("/amiga/cmd_vel", Twist, queue_size=10)

    # initialize the publishing node
    rospy.init_node("amiga_vel_pub", anonymous=True)

    # define how many times per second
    # will the data be published
    # let's say 10 times/second or 10Hz
    rate = rospy.Rate(10)

    # to keep publishing as long as the core is running
    while not rospy.is_shutdown():

        # create a Twist object
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.25

        # you could simultaneously display the data
        # on the terminal and to the log file
        rospy.loginfo(cmd_vel)

        # publish the data to the topic using publish()
        pub.publish(cmd_vel)

        # keep a buffer based on the rate defined earlier
        rate.sleep()
