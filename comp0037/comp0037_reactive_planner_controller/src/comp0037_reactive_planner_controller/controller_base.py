#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
import pickle
import math
import Observer
# This is the base class of the controller which moves the robot to its goal.
# could do.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupancy grid. This is dynamically updated as a result of new map
        # information becoming available.
        self.occupancyGrid = occupancyGrid

        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)

        # This flag says if the current goal should be aborted
        self.abortCurrentGoal = False

        # Run the observer
        Observer.runMyThread(self)






    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation

        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose


        # Check the format - should be float or round up to another position?
        with open('/home/ros_user/catkin_ws/src/comp0037/comp0037_explorer/src/comp0037_explorer/position.txt', 'w') as file:
            file.write(str(position.x)+'\n')
            file.write(str(position.y))


    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # If set to true, the robot should abort driving to the current goal.
    def stopDrivingToCurrentGoal(self):
        self.abortCurrentGoal = True

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    def stopRobot(self):
        stopMessage = Twist()
        self.velocityPublisher.publish(stopMessage)

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):


        self.abortCurrentGoal = False
        self.plannerDrawer = plannerDrawer

        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')

        # start = time.time() # START
        # self.total_distance = 0.0
        # self.total_angle = 0.0
        # self.total_angle_asked = 0.0

        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)

            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])

            if self.abortCurrentGoal is True:
                self.stopRobot()
                return False

            if self.driveToWaypoint(waypoint) is False:
                self.stopRobot()
                return False

            # Handle ^C
            if rospy.is_shutdown() is True:
                return False



        # f = open('../recorded_data.csv', "a")
        # f.write("{}, {},".format(path.waypoints[0].coords, path.waypoints[len(path.waypoints)-1].coords))
        # f.write("{},".format(str(time.time() - start)))
        # f.write("{},".format(str(self.total_distance)))
        # f.write("{},".format(str(self.total_angle*360.0/6.28)))
        # f.write("{}\n".format(str(self.total_angle_asked*360.0/6.28)))
        # f.close()

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')

        # Finish off by rotating the robot to the final configuration
        return self.rotateToGoalOrientation(goalOrientation)
 
