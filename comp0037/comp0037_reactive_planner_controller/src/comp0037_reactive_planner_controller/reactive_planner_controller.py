# This class manages the key logic for the reactive planner and
# controller. This monitors the the robot mtion.

import rospy
import threading
from cell import CellLabel
from planner_controller_base import PlannerControllerBase
from comp0037_mapper.msg import *
from comp0037_reactive_planner_controller.dijkstra_planner import DijkstraPlanner


class ReactivePlannerController(PlannerControllerBase):

    def __init__(self, occupancyGrid, planner, controller):
        PlannerControllerBase.__init__(self, occupancyGrid, planner, controller)

        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.gridUpdateLock =  threading.Condition()

    def mapUpdateCallback(self, mapUpdateMessage):

        # Update the occupancy grid and search grid given the latest map update
        self.gridUpdateLock.acquire()
        self.occupancyGrid.updateGridFromVector(mapUpdateMessage.occupancyGrid)
        self.planner.handleChangeToOccupancyGrid()
        self.gridUpdateLock.release()

        # If we are not currently following any route, drop out here.
        if self.currentPlannedPath is None:
            return

        self.checkIfPathCurrentPathIsStillGood()

    def checkIfPathCurrentPathIsStillGood(self):

        # This methods needs to check if the current path, whose
        # waypoints are in self.currentPlannedPath, can still be
        # traversed

        # Iterate through the points of the current path
        # # Very basic implementation: simply calls the planner everytime an obstacle is found
        # for waypoint in self.currentPlannedPath.waypoints:
        #     # If the cell is occuppied, find a new path
        #     if self.occupancyGrid.getCell(waypoint.coords[0], waypoint.coords[1]) == 1.0:
        #         self.controller.stopDrivingToCurrentGoal()
        #     else:
        #         continue

        # Implementation #2: the idea is to keep driving until the cell before the occupied one. #
        # ----------------------------- I AM HERE NOW -----------------------------------#

        reached = False
        for waypoint in reversed(self.currentPlannedPath.waypoints):
            # If the cell is occuppied, find a new path
            if self.occupancyGrid.getCell(waypoint.coords[0], waypoint.coords[1]) == 1.0:
                pose = self.controller.getCurrentPose()
                start = (pose.x, pose.y)
                currentCell = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
                with open('log.txt','a+') as file:
                    file.write('-'*40)
                    file.write('CURRENT CELL POSITION: {}'.format(currentCell))
                    file.write('WAYPOINT OCUPPIED: {}'.format(waypoint.coords))


                # Calculate new path in the background
                # Create a new planner object
                # goalCellCoords = self.currentPlannedPath.waypoints[-1].coords
                # self.planner_new = DijkstraPlanner('Dijkstra', self.occupancyGrid)
                # self.planner_new.showGraphics = False
                # self.planner_new.search(currentCell, goalCellCoords)
                # self.newPlannedPath = self.planner_new.extractPathToGoal()

                # If the new travel cost is 50% more, don't keep going
                # if abs(int(self.newPlannedPath.travelCost) - int(self.currentPlannedPath.travelCost))/int(self.currentPlannedPath.travelCost) < 0.2:
                while (reached == False):
                    pose = self.controller.getCurrentPose()
                    start = (pose.x, pose.y)
                    with open('log.txt', 'a+') as file:
                        file.write('Current poisiton: {}'.format(self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)))
                    currentCell = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
                    if (abs(currentCell[0] - waypoint.coords[0]) < 4) and (abs(currentCell[1] - waypoint.coords[1]) < 4):
                        print('*****************'*20)
                        reached = True

                self.controller.stopDrivingToCurrentGoal()
                break
                # else:
                #     self.controller.stopDrivingToCurrentGoal()


            else:
                continue

                
        # If the route is not viable any more, call
        # self.controller.stopDrivingToCurrentGoal()

    
    def driveToGoal(self, goal):

        # Get the goal coordinate in cells
        goalCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((goal.x,goal.y))

        # Reactive planner main loop - keep iterating until the goal is reached or the robot gets
        # stuck.

        goalReached = False
        
        while (goalReached is False) & (rospy.is_shutdown() is False):

            # Set the start conditions to the current position of the robot
            pose = self.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)

            print 'Planning a new path: start=' + str(start) + '; goal=' + str(goal)
            
            # Plan a path using the current occupancy grid
            self.gridUpdateLock.acquire()
            pathToGoalFound = self.planner.search(startCellCoords, goalCellCoords)
            self.gridUpdateLock.release()

            # If we can't reach the goal, give up and return
            if pathToGoalFound is False:
                rospy.logwarn("Could not find a path to the goal at (%d, %d)", \
                              goalCellCoords[0], goalCellCoords[1])
                return False
            
            # Extract the path
            self.currentPlannedPath = self.planner.extractPathToGoal()

            # Drive along the path towards the goal. This returns True
            # if the goal was successfully reached. The controller
            # should stop the robot and return False if the
            # stopDrivingToCurrentGoal method is called.
            goalReached = self.controller.drivePathToGoal(self.currentPlannedPath, \
                                                          goal.theta, self.planner.getPlannerDrawer())

            rospy.logerr('goalReached=%d', goalReached)

        return goalReached
            
            
