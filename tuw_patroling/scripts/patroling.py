#!/usr/bin/env python

# license removed for brevity
import rospy
from tuw_multi_robot_msgs.msg import RobotInfo
from geometry_msgs.msg import PoseStamped


class Patroling:
    def __init__(self):
        self.counter = 0
        self.goal_poses_index = 0
        self.goal_poses = []
        self.number_of_goals = rospy.get_param("~number_of_goals")
        self.get_goal_poses()
        self.next_goal_pose = self.goal_poses[self.goal_poses_index]
        self.threshold_in_meters = 0.5
        self.pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('robot_info', RobotInfo, self.robot_info_callback)
        self.rate = rospy.Rate(2)  # 10hz
        self.initial_publishing()

    def get_goal_pose_from_cfg(self, goal_name_identifier):
        new_pose = PoseStamped()
        goal_position_x = '~goal_position_x_' + goal_name_identifier
        new_pose.pose.position.x = rospy.get_param(goal_position_x)

        goal_poisition_y = '~goal_position_y_' + goal_name_identifier
        new_pose.pose.position.y = rospy.get_param(goal_poisition_y)

        goal_orientation_z = '~goal_orientation_z_' + goal_name_identifier
        new_pose.pose.orientation.z = rospy.get_param(goal_orientation_z)

        goal_orientation_w = '~goal_orientation_w_' + goal_name_identifier
        new_pose.pose.orientation.w = rospy.get_param(goal_orientation_w)

        return new_pose

    def is_it_near_the_goal(self, robot_x, robot_y, goal_x, goal_y):
        if robot_x < (goal_x + self.threshold_in_meters) and (robot_x > (goal_x - self.threshold_in_meters)):
            if robot_y < (goal_y + self.threshold_in_meters) and (robot_y > (goal_y - self.threshold_in_meters)):
                return True
        return False

    def set_next_goal_pose(self):
        self.counter += 1
        if self.counter > 4:
            self.counter = 0
            self.goal_poses_index += 1
            self.goal_poses_index %= self.number_of_goals
            self.next_goal_pose = self.goal_poses[self.goal_poses_index]
            self.next_goal_pose.header.stamp = rospy.get_rostime()
            self.next_goal_pose.header.frame_id = 'map'
            self.next_goal_pose.pose.orientation.z = 0

            rospy.sleep(1.0)
            self.pub.publish(self.next_goal_pose)

    def robot_info_callback(self, robot_info):
        robot_x = robot_info.pose.pose.position.x
        robot_y = robot_info.pose.pose.position.y
        goal_x = self.next_goal_pose.pose.position.x
        goal_y = self.next_goal_pose.pose.position.y

        if self.is_it_near_the_goal(robot_x, robot_y, goal_x, goal_y):
            self.set_next_goal_pose()

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def initial_publishing(self):
        self.next_goal_pose.header.stamp = rospy.get_rostime()
        self.next_goal_pose.header.frame_id = 'map'
        self.next_goal_pose.pose.orientation.z = 0

        rospy.sleep(1.0)
        self.pub.publish(self.next_goal_pose)
        rospy.spin()


    def get_goal_poses(self):
        for i in range(65, 65 + self.number_of_goals):
            goal_pose = self.get_goal_pose_from_cfg(chr(i))
            self.goal_poses.append(goal_pose)


if __name__ == '__main__':
    try:
        rospy.init_node('patroling', anonymous=False)
        Patroling_node = Patroling()
    except rospy.ROSInterruptException:
        pass
