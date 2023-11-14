import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point



def goal_publisher():

    goal_msg = Point()
    goal_msg.x = float(input("x_goal?"))
    goal_msg.z = float(input("z_goal?"))
    goal_msg.y = float(input("y_goal?"))

    return goal_msg

if __name__ == '__main__':
    rospy.init_node("goal_node")
    goal = rospy.Publisher("Goal", Point, queue_size=10)
    rate = rospy.Rate(1)
    msg = goal_publisher()
    while not rospy.is_shutdown():
        goal.publish(msg)
        rate.sleep()