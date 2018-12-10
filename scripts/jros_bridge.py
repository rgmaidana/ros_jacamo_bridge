#!/usr/bin/env python

# ROS API
import rospy

# ROS messages
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

# Current robot pose
robotPose = PoseWithCovarianceStamped()

# Occupancy grid map
occupancyMap = OccupancyGrid()

# Callback for robot pose subscriber
def poseCallback(msg):
    global robotPose
    robotPose = msg

# Callback for occupancy grid map
def occupancyCallback(msg):
    global occupancyMap
    occupancyMap = msg

# ROS node main function
def node():
    # Initialize node
    rospy.init_node('jros_bridge', anonymous=True)

    # Publisher for current robot occupancy as a string (as understood by JROS)
    occStrPub = rospy.Publisher('/JROS/occupancy', String, queue_size=10)

    # Subscribers
    poseTopic = rospy.get_param('/jros_bridge/pose_topic', '/amcl_pose')
    rospy.Subscriber(poseTopic, PoseWithCovarianceStamped, poseCallback)
    occupancyTopic = rospy.get_param('/jros_bridge/occupancy_topic', '/move_base/global_costmap/costmap')
    rospy.Subscriber(occupancyTopic, OccupancyGrid, occupancyCallback)

    # Wait for messages to come in
    rospy.loginfo("[JROS Bridge] Waiting for occupancy grid map")
    while occupancyMap.info.width == 0:
        pass

    # Publish rate (10 Hz)
    rate = rospy.Rate(10)

    # Run until node is shut down
    rospy.loginfo("[JROS Bridge] Publishing occupancy cost data at %s topic" % '/JROS/occupancy')
    while not rospy.is_shutdown():
        # Get current occupancy cost based on robot's position in the world
        x, y = robotPose.pose.pose.position.x, robotPose.pose.pose.position.y
        gridX = int((x - int(occupancyMap.info.origin.position.x)) / occupancyMap.info.resolution)
        gridY = int((y - int(occupancyMap.info.origin.position.y)) / occupancyMap.info.resolution)
        occupancyCost = occupancyMap.data[gridY * occupancyMap.info.width + gridX]

        # Wrap the occupancy cost in a String message
        occStrMsg = String()
        occStrMsg.data = '%d' % occupancyCost

        # Publish the occupancy cost
        occStrPub.publish(occStrMsg)

        # Sleep
        rate.sleep()

# Python "main" function
if __name__ == "__main__":
    node()