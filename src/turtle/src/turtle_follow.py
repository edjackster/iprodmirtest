#!/usr/bin/python
import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class PoliceTurtle:
    def __init__(this, turtleThief, turtleCop, policeTurtleSpeed):
        this.turtleThief = turtleThief
        this.turtleCop = turtleCop
        this.policeTurtleSpeed = policeTurtleSpeed
        this.pose1 = Pose()
        this.pose2 = Pose()
        rospy.Subscriber("/" + this.turtleThief + "/pose", Pose, this.reverse_message1)
        rospy.Subscriber("/" + this.turtleCop + "/pose", Pose, this.reverse_message2)
        this.pub = rospy.Publisher("/" + this.turtleCop + "/cmd_vel", Twist, queue_size=10)  

        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(random.uniform(0, 11), random.uniform(0, 11), 0, turtleCop)
        policePoseMessage = rospy.wait_for_message("/" + this.turtleCop + "/pose", Pose)
        this.pose2.x = policePoseMessage.x
        this.pose2.y = policePoseMessage.y

    def reverse_message1(this, data):
        this.pose1 = data

    def reverse_message2(this,data):
        this.pose2 = data

    def follow(this):
        while not rospy.is_shutdown():
            xDistance = this.pose1.x - this.pose2.x
            yDistance = this.pose1.y - this.pose2.y

            distance = max(((xDistance)**2 + (yDistance)**2)**0.5-1, 0)
            corner = math.atan2(yDistance, xDistance)
            cornerDelta = min(corner - this.pose2.theta, math.pi)

            cmd_vel = Twist()
            cmd_vel.linear.x = min(this.policeTurtleSpeed * distance, this.policeTurtleSpeed)
            cmd_vel.angular.z = 4.0 * cornerDelta

            this.pub.publish(cmd_vel)

            rospy.sleep(0.2)


if __name__ == '__main__':
    rospy.init_node('police_turtle_node')

    policeTurtleSpeed = rospy.get_param('~police_speed')

    turtleThief = rospy.get_param('~turtle1_name', 'turtle1')
    turtleCop = rospy.get_param('~turtle2_name', 'turtle2')

    policeTurtle = PoliceTurtle(turtleThief, turtleCop, policeTurtleSpeed)
    policeTurtle.follow()

