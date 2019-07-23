#!/usr/bin/env python

# license removed for brevity
import rospy
from tuw_multi_robot_msgs.msg import RobotInfo
from geometry_msgs.msg import PoseStamped


def get_goal_pose_from_cfg(goal_name_identifier):
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


rospy.init_node('patroling', anonymous=True)
counter = 0
goal_poses_index = 0
goal_poseA = get_goal_pose_from_cfg('A')
goal_poseB = get_goal_pose_from_cfg('B')
goal_poses = [goal_poseA, goal_poseB]
next_goal_pose = goal_poseA



def is_it_near_the_goal(robot_x, robot_y, goal_x, goal_y, threshold_in_meters):
    if robot_x < (goal_x + threshold_in_meters) and (robot_x > (goal_x - threshold_in_meters)):
        if robot_y < (goal_y + threshold_in_meters) and (robot_y > (goal_y - threshold_in_meters)):
            return True
    return False


def set_next_goal_pose():
    global counter, goal_poses_index, next_goal_pose
    counter += 1
    if counter > 4:
        counter = 0
        goal_poses_index += 1
        goal_poses_index %= 2
        next_goal_pose = goal_poses[goal_poses_index]


def robot_info_callback(data):
    robot_info = data
    robot_x = robot_info.pose.pose.position.x
    robot_y = robot_info.pose.pose.position.y
    goal_x = next_goal_pose.pose.position.x
    goal_y = next_goal_pose.pose.position.y

    if is_it_near_the_goal(robot_x, robot_y, goal_x, goal_y, 0.5):
        set_next_goal_pose()

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def talker():
    pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
    rospy.Subscriber("robot_info", RobotInfo, robot_info_callback)

    rate = rospy.Rate(2)  # 10hz
    while not rospy.is_shutdown():
        next_goal_pose.header.stamp = rospy.get_rostime()
        next_goal_pose.header.frame_id = 'map'
        pub.publish(next_goal_pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
