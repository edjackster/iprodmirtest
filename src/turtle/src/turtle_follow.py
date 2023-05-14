#!/usr/bin/python
import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn




class TurtleFollower: #####
    def __init__(self, turtleOne, turtleTwo, speed_of_stalker):
        self.turtleOne = turtleOne
        self.turtleTwo = turtleTwo
        self.speed_of_stalker = speed_of_stalker
        self.pose1 = Pose()
        self.pose2 = Pose()
        rospy.Subscriber("/" + self.turtleOne + "/pose", Pose, self.reverse_message1)
        rospy.Subscriber("/" + self.turtleTwo + "/pose", Pose, self.reverse_message2)
        self.pub = rospy.Publisher("/" + self.turtleTwo + "/cmd_vel", Twist, queue_size=10)  

        # spawn a turtle
        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(random.uniform(0, 11), random.uniform(0, 11), 0, turtleTwo)
        pose2_wf_message = rospy.wait_for_message("/" + self.turtleTwo + "/pose", Pose)
        self.pose2.x = pose2_wf_message.x
        self.pose2.y = pose2_wf_message.y

    def reverse_message1(self, data):
        self.pose1 = data

    def reverse_message2(self,data):
        self.pose2 = data

    def follow(self):
        while not rospy.is_shutdown():
            x_pose1_pose2 = self.pose1.x - self.pose2.x
            y_pose1_pose2 = self.pose1.y - self.pose2.y

            # distance calculation 
            range = ((x_pose1_pose2)**2 + (y_pose1_pose2)**2)**0.5
            corner = math.atan2(y_pose1_pose2, x_pose1_pose2)
            corner_pose1_pose2 = corner - self.pose2.theta

            cmd_vel = Twist()
            cmd_vel.linear.x = self.speed_of_stalker * range
            cmd_vel.angular.z = 4.0 * corner_pose1_pose2

            self.pub.publish(cmd_vel)

            rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('turtle_follower_node')

    speed_of_stalker = rospy.get_param('~follower_speed')

    turtleOne = rospy.get_param('~turtle1_name', 'turtle1')
    turtleTwo = rospy.get_param('~turtle2_name', 'turtle2')

    follower = TurtleFollower(turtleOne, turtleTwo, speed_of_stalker)
    follower.follow()

