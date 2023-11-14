import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

x_goal = 0
z_goal = 0
y_goal = 0

ux = 0
uy = 0
uz = 0

ex = 0
ez = 0
ey = 0

exacc = 0
ezacc = 0
eyacc = 0

twi_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

def ground_truth_callback(msg):
    global x_pos
    global z_pos
    global y_pos
    global yaw

    x_pos = msg.pose.pose.position.x
    z_pos = msg.pose.pose.position.z
    y_pos = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    #print("Received Ground Truth State: {}".format(msg))
    

def goal_callback(msg):
    global x_goal
    global z_goal
    global y_goal
    x_goal = msg.x
    z_goal = msg.z
    y_goal = msg.y
    
    print("Received Goal: {}".format(msg))
    


def pid_callback(msg):
    global ux, uy, uz, exacc, ezacc, eyacc, ex, ez, ey
    dt = 0.2
    kp = 0.5
    ki = 0.0
    kd = 0.0


    dex = ((x_goal - x_pos) - ex)/dt
    dez = ((z_goal - z_pos) - ez)/dt
    dey = ((y_goal - y_pos) - ey)/dt

    ex = x_goal - x_pos
    ez = z_goal - z_pos
    ey = y_goal - y_pos
    
    exacc = exacc + ex * dt
    ezacc = ezacc + ez * dt
    eyacc = eyacc + ey * dt

    ux_w = kp * ex + ki * exacc + kd * dex
    uz_w = kp * ez + ki * ezacc + kd * dez
    uy_w = kp * ey + ki * eyacc + kd * dey

    #np.array[[ux],[uy],[uz]] = np.array[[np.cos(yaw), np.sin(yaw), 0], [-np.sin(yaw), np.cos(yaw),0], [0, 0, 1]]@np.array[[ux_b], [uy_b], [uz_b]]
    ux = np.cos(yaw) * ux_w + np.sin(yaw) * uy_w
    uy = -np.sin(yaw) * ux_w + np.cos(yaw) * uy_w
    uz = uz_w  

    t = Twist()
    t.linear.x = ux
    t.linear.z = uz
    t.linear.y = uy
    print(t.linear.z)

    twi_pub.publish(t)



def listener():
    rospy.init_node("positionGoal_sub_node")
    goal_sub = rospy.Subscriber("/Goal", Point, goal_callback)
    pos_sub = rospy.Subscriber("/ground_truth/state", Odometry, ground_truth_callback)
    twi_sub= rospy.Timer(rospy.Duration(0.2), pid_callback)



if __name__ == '__main__':
    listener()
    rospy.spin()