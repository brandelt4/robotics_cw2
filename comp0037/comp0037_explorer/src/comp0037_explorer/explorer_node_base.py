import rospy
import threading
import math
import copy
import pickle

from comp0037_mapper.msg import *
from comp0037_mapper.srv import *
from comp0037_reactive_planner_controller.srv import *
from comp0037_reactive_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_reactive_planner_controller.grid_drawer import OccupancyGridDrawer
from geometry_msgs.msg  import Twist

class ExplorerNodeBase(object):

    def __init__(self):
        rospy.init_node('explorer')

        # Get the drive robot service
        rospy.loginfo('Waiting for service drive_to_goal')
        rospy.wait_for_service('drive_to_goal')
        self.driveToGoalService = rospy.ServiceProxy('drive_to_goal', Goal)
        rospy.loginfo('Got the drive_to_goal service')

        self.waitForGoal =  threading.Condition()
        self.waitForDriveCompleted =  threading.Condition()
        self.goal = None

        # Subscribe to get the map update messages
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.noMapReceived = True

        # Clear the map variables
        self.occupancyGrid = None
        self.deltaOccupancyGrid = None

        self.counter = 0
        self.frontier = []
        self.previousFrontier = []

        # Flags used to control the graphical output. Note that we
        # can't create the drawers until we receive the first map
        # message.
        self.showOccupancyGrid = rospy.get_param('show_explorer_occupancy_grid', True)
        self.showDeltaOccupancyGrid = rospy.get_param('show_explorer_delta_occupancy_grid', True)
        self.occupancyGridDrawer = None
        self.deltaOccupancyGridDrawer = None
        self.visualisationUpdateRequired = False

        # Request an initial map to get the ball rolling
        rospy.loginfo('Waiting for service request_map_update')
        rospy.wait_for_service('request_map_update')
        mapRequestService = rospy.ServiceProxy('request_map_update', RequestMapUpdate)
        mapUpdate = mapRequestService(True)

        while mapUpdate.initialMapUpdate.isPriorMap is True:
            self.kickstartSimulator()
            mapUpdate = mapRequestService(True)
            
        self.mapUpdateCallback(mapUpdate.initialMapUpdate)
        
    def mapUpdateCallback(self, msg):
        rospy.loginfo("map update received")
        
        # If the occupancy grids do not exist, create them
        if self.occupancyGrid is None:
            self.occupancyGrid = OccupancyGrid.fromMapUpdateMessage(msg)
            self.deltaOccupancyGrid = OccupancyGrid.fromMapUpdateMessage(msg)

        # Update the grids
        self.occupancyGrid.updateGridFromVector(msg.occupancyGrid)
        self.deltaOccupancyGrid.updateGridFromVector(msg.deltaOccupancyGrid)
        
        # Update the frontiers
        self.updateFrontiers()

        # Flag there's something to show graphically
        self.visualisationUpdateRequired = True

    # This method determines if a cell is a frontier cell or not. A
    # frontier cell is open and has at least one neighbour which is
    # unknown.
    def isFrontierCell(self, x, y):

        # Check the cell to see if it's open
        if self.occupancyGrid.getCell(x, y) != 0:
            return False

        # Check the neighbouring cells; if at least one of them is unknown, it's a frontier
        return self.checkIfCellIsUnknown(x, y, -1, -1) | self.checkIfCellIsUnknown(x, y, 0, -1) \
            | self.checkIfCellIsUnknown(x, y, 1, -1) | self.checkIfCellIsUnknown(x, y, 1, 0) \
            | self.checkIfCellIsUnknown(x, y, 1, 1) | self.checkIfCellIsUnknown(x, y, 0, 1) \
            | self.checkIfCellIsUnknown(x, y, -1, 1) | self.checkIfCellIsUnknown(x, y, -1, 0)
            
    def checkIfCellIsUnknown(self, x, y, offsetX, offsetY):
        newX = x + offsetX
        newY = y + offsetY
        return (newX >= 0) & (newX < self.occupancyGrid.getWidthInCells()) \
            & (newY >= 0) & (newY < self.occupancyGrid.getHeightInCells()) \
            & (self.occupancyGrid.getCell(newX, newY) == 0.5)


    def isThePointThere(self, idx, frontiers):

        inFrontier = False
        whereIsIdx = None
        for f, frontier in enumerate(frontiers):
            if idx in frontier:
                inFrontier = True
                whereIsIdx = f
            else:
                continue


        return inFrontier, whereIsIdx




    # You should provide your own implementation of this method which
    # maintains and updates the frontiers.  The method should return
    # True if any outstanding frontiers exist. If the method returns
    # False, it is assumed that the map is completely explored and the
    # explorer will exit.

    def updateFrontiers(self):
        self.previousFrontier = copy.copy(self.frontier)

        # If the first time, check all cells
        if self.counter == 1:
            print('.'*80)
            print('First time updating frontiers')
            print('.'*80)
            for x in range(0, self.occupancyGrid.getWidthInCells()):
                for y in range(0, self.occupancyGrid.getHeightInCells()):
                    if self.isFrontierCell(x, y) == True:
                        self.frontier.append((x,y))
                    else:
                        continue

        # Otherwise, only check the Delta Grid
        else:

            # First, check all current frontier points - are they still frontier points?
            for idx, cell_coords in enumerate(self.frontier):
                if self.isFrontierCell(cell_coords[0], cell_coords[1]) is True:
                    continue
                else:
                    self.frontier.pop(idx)

            # Second, check the changed points from Delta Grid and add new frontier points
            for x in range(0, self.deltaOccupancyGrid.getWidthInCells()):
                for y in range(0, self.deltaOccupancyGrid.getHeightInCells()):
                    # If the cell was changed and not in the frontier list already
                    if (self.deltaOccupancyGrid.getCell(x,y) == 1.0) and not ((x,y) in self.frontier):
                        if self.isFrontierCell(x,y) is True:
                            self.frontier.append((x,y))

        # Write frontiers into a pickle
        with open('/home/ros_user/catkin_ws/src/comp0037/comp0037_reactive_planner_controller/src/comp0037_reactive_planner_controller/frontier.pkl', 'w+b') as file:
            pickle.dump(self.frontier, file)


        # Now that we have all frontier points, let's split into multiple frontiers

        # Stores indices of frontiers from self.frontier
        self.frontiers = []
        self.frontiers.append([0])

        temp_blacklist = []
        temp_blacklist.append(0)

        for idx, frontierPoint in enumerate(self.frontier):
            if idx in temp_blacklist:
                continue
            temp_blacklist.append(idx)
            
            for idx2, frontierPoint2 in enumerate(self.frontier):

                if idx2 in temp_blacklist:
                    continue

                # If they are next to each other
                if (frontierPoint[0] + 1, frontierPoint[1] + 1) == frontierPoint2 \
                    or (frontierPoint[0] + 1, frontierPoint[1]) == frontierPoint2 \
                    or (frontierPoint[0] + 1, frontierPoint[1]-1) == frontierPoint2 \
                    or (frontierPoint[0] - 1, frontierPoint[1] + 1) == frontierPoint2 \
                    or (frontierPoint[0] - 1, frontierPoint[1]) == frontierPoint2 \
                    or (frontierPoint[0] - 1, frontierPoint[1]-1) == frontierPoint2 \
                    or (frontierPoint[0], frontierPoint[1] + 1) == frontierPoint2 \
                    or (frontierPoint[0], frontierPoint[1] - 1) == frontierPoint2:


                    # Is idx already in self.frontiers?
                    idxInFrontiers = False
                    for f, frontier in enumerate(self.frontiers):
                        if idx in frontier:
                            whereIsIdx = f
                            idxInFrontiers = True
                        else:
                            continue

                    # Append idx2 to where idx is
                    if idxInFrontiers:
                        self.frontiers[whereIsIdx].append(idx2)
                    else:
                        self.frontiers.append([idx, idx2])


                    # If idx2 appended, add it to the blacklist:
                    temp_blacklist.append(idx2)






        #
        # for idx, frontierPoint in enumerate(self.frontier):
        #
        #     there, whereIsIdx = self.isThePointThere(idx, self.frontiers)
        #
        #     if there:
        #
        #         if (frontierPoint[0] + 1, frontierPoint[1] + 1) in self.frontier:
        #             point = (frontierPoint[0] + 1, frontierPoint[1] + 1)
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0] + 1, frontierPoint[1]) in self.frontier:
        #             point = (frontierPoint[0] + 1, frontierPoint[1])
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0] + 1, frontierPoint[1] - 1) in self.frontier:
        #             point = (frontierPoint[0] + 1, frontierPoint[1] - 1)
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0] - 1, frontierPoint[1] + 1) in self.frontier:
        #             point = (frontierPoint[0] - 1, frontierPoint[1] + 1)
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0] - 1, frontierPoint[1]) in self.frontier:
        #             point = (frontierPoint[0] - 1, frontierPoint[1])
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #             self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0] - 1, frontierPoint[1] - 1) in self.frontier:
        #             point = (frontierPoint[0] - 1, frontierPoint[1] - 1)
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0], frontierPoint[1] + 1) in self.frontier:
        #             point = (frontierPoint[0], frontierPoint[1] + 1)
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #         if (frontierPoint[0], frontierPoint[1] - 1) in self.frontier:
        #             point = (frontierPoint[0], frontierPoint[1] - 1)
        #             point_index = self.frontier.index(point)
        #             p_there, p_idx = self.isThePointThere(point_index, self.frontiers)
        #
        #             if p_there:
        #                 continue
        #             else:
        #                 self.frontiers[whereIsIdx].append(point_index)
        #
        #     else:
        #
        #         self.frontiers.append([idx])
        #
        #         # If self.frontier contains a point next to idx
        #         if (frontierPoint[0] + 1, frontierPoint[1] + 1) in self.frontier:
        #             point = (frontierPoint[0] + 1, frontierPoint[1] + 1)
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0] + 1, frontierPoint[1]) in self.frontier:
        #             point = (frontierPoint[0] + 1, frontierPoint[1])
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0] + 1, frontierPoint[1] - 1) in self.frontier:
        #             point = (frontierPoint[0] + 1, frontierPoint[1] - 1)
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0] - 1, frontierPoint[1] + 1) in self.frontier:
        #             point = (frontierPoint[0] - 1, frontierPoint[1] + 1)
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0] - 1, frontierPoint[1]) in self.frontier:
        #             point = (frontierPoint[0] - 1, frontierPoint[1])
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0] - 1, frontierPoint[1] - 1) in self.frontier:
        #             point = (frontierPoint[0] - 1, frontierPoint[1] - 1)
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0], frontierPoint[1] + 1) in self.frontier:
        #             point = (frontierPoint[0], frontierPoint[1] + 1)
        #             self.frontiers[-1].append(self.frontier.index(point))
        #
        #         if (frontierPoint[0], frontierPoint[1] - 1) in self.frontier:
        #             point = (frontierPoint[0], frontierPoint[1] - 1)
        #             self.frontiers[-1].append(self.frontier.index(point))





        # # Iterate through the points and find which points are next to each other
        # # If points are next to each other, group them into separate frontiers
        # for idx, frontierPoint in enumerate(self.frontier):
        #     for idx2, frontierPoint2 in enumerate(self.frontier):
        #
        #         if idx == idx2:
        #             continue
        #
        #         # If they are next to each other
        #         if (frontierPoint[0] + 1, frontierPoint[1] + 1) == frontierPoint2 \
        #             or (frontierPoint[0] + 1, frontierPoint[1]) == frontierPoint2 \
        #             or (frontierPoint[0] + 1, frontierPoint[1]-1) == frontierPoint2 \
        #             or (frontierPoint[0] - 1, frontierPoint[1] + 1) == frontierPoint2 \
        #             or (frontierPoint[0] - 1, frontierPoint[1]) == frontierPoint2 \
        #             or (frontierPoint[0] - 1, frontierPoint[1]-1) == frontierPoint2 \
        #             or (frontierPoint[0], frontierPoint[1] + 1) == frontierPoint2 \
        #             or (frontierPoint[0], frontierPoint[1] - 1) == frontierPoint2:
        #
        #             # Is idx already in self.frontiers?
        #             idxInFrontiers = False
        #             for f, frontier in enumerate(self.frontiers):
        #                 if idx in frontier:
        #                     whereIsIdx = f
        #                     idxInFrontiers = True
        #                 else:
        #                     continue
        #
        #             # Is idx2 already in self.frontiers?
        #             idx2InFrontiers = False
        #             for f2, frontier in enumerate(self.frontiers):
        #                 if idx2 in frontier:
        #                     whereIsIdx2 = f2
        #                     idx2InFrontiers = True
        #                 else:
        #                     continue
        #
        #
        #             # If idx in self.frontiers, add idx2 to that frontier:
        #             if idxInFrontiers:
        #                 # If idx2 is already in that frontier, skip!
        #                 if idx2 in self.frontiers[whereIsIdx]:
        #                     continue
        #
        #                 # If idx2 was in self.frontiers already, pop it and then add to idx
        #                 elif idx2InFrontiers:
        #                     self.frontiers[whereIsIdx2].pop(self.frontiers[whereIsIdx2].index(idx2))
        #                     self.frontiers[whereIsIdx].append(idx2)
        #                 else:
        #                     self.frontiers[whereIsIdx].append(idx2)
        #
        #             # Otherwise, add idx as a new frontier
        #             else:
        #                 if idx2InFrontiers:
        #                     self.frontiers[whereIsIdx2].append(idx)
        #                 else:
        #                     self.frontiers.append([idx, idx2])






        with open('/home/ros_user/catkin_ws/frontiers.txt', 'w+') as file:
            file.write(str(self.frontier))
            file.write('\n\n')


            for frontier in self.frontiers:
                file.write(str(frontier))
                file.write('\n')

        print("Frontiers updated #{}".format(self.counter))
        self.counter += 1

        if self.frontier == self.previousFrontier:
            return False
        else:
            return True


    def chooseNewDestination(self):
        raise NotImplementedError()

    def destinationReached(self, goalReached):
        raise NotImplementedError()

    def updateVisualisation(self):

        # If we don't need to do an update, simply flush the graphics
        # to make sure everything appears properly in VNC
        
        if self.visualisationUpdateRequired is False:

            if self.occupancyGridDrawer is not None:
                self.occupancyGridDrawer.flushAndUpdateWindow()

            if self.deltaOccupancyGridDrawer is not None:
                self.deltaOccupancyGridDrawer.flushAndUpdateWindow()

            return
                

        # Update the visualisation; note that we can only create the
        # drawers here because we don't know the size of the map until
        # we get the first update from the mapper.
        if self.showOccupancyGrid is True:
            if self.occupancyGridDrawer is None:
                windowHeight = rospy.get_param('maximum_window_height_in_pixels', 600)
                self.occupancyGridDrawer = OccupancyGridDrawer('Explorer Node Occupancy Grid',\
                                                               self.occupancyGrid, windowHeight)
	        self.occupancyGridDrawer.open()
	    self.occupancyGridDrawer.update()

        if self.showDeltaOccupancyGrid is True:
            if self.deltaOccupancyGridDrawer is None:
                windowHeight = rospy.get_param('maximum_window_height_in_pixels', 600)
                self.deltaOccupancyGridDrawer = OccupancyGridDrawer('Explorer Node Delta Occupancy Grid',\
                                                                    self.deltaOccupancyGrid, windowHeight)
	        self.deltaOccupancyGridDrawer.open()
	    self.deltaOccupancyGridDrawer.update()

        self.visualisationUpdateRequired = False

    def sendGoalToRobot(self, goal):
        rospy.logwarn("Sending the robot to " + str(goal))
        try:
            response = self.driveToGoalService(goal[0], goal[1], 0)
            return response.reachedGoal
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    # Special case. If this is the first time everything has
    # started, stdr needs a kicking to generate the first
    # laser message. Telling it to take a very small movement appears to be sufficient
    def kickstartSimulator(self):
        velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)
        velocityMessage = Twist()
        velocityPublisher.publish(velocityMessage)
        rospy.sleep(1)
            
    class ExplorerThread(threading.Thread):
        def __init__(self, explorer):
            threading.Thread.__init__(self)
            self.explorer = explorer
            self.running = False
            self.completed = False;


        def isRunning(self):
            return self.running

        def hasCompleted(self):
            return self.completed

        def run(self):

            self.running = True

            while (rospy.is_shutdown() is False) & (self.completed is False):

                # Special case. If this is the first time everything
                # has started, stdr needs a kicking to generate laser
                # has started, stdr needs a kicking to generate laser
                # messages. To do this, we get the robot to

                self.explorer.updateFrontiers()

                # Create a new robot waypoint if required
                newDestinationAvailable, newDestination = self.explorer.chooseNewDestination()
                self.previousDestination = copy.copy(newDestination)

                # Convert to world coordinates, because this is what the robot understands
                if newDestinationAvailable is True:
                    print('newDestination = ' + str(newDestination))
                    newDestinationInWorldCoordinates = self.explorer.occupancyGrid.getWorldCoordinatesFromCellCoordinates(
                        newDestination)
                    attempt = self.explorer.sendGoalToRobot(newDestinationInWorldCoordinates)
                    self.explorer.destinationReached(newDestination, attempt)
                else:
                    self.completed = True

    def run(self):

        explorerThread = ExplorerNodeBase.ExplorerThread(self)

        keepRunning = True
        
        while (rospy.is_shutdown() is False) & (keepRunning is True):

            rospy.sleep(0.1)
            
            self.updateVisualisation()

            if self.occupancyGrid is None:
                continue

            if explorerThread.isRunning() is False:
                explorerThread.start()

            if explorerThread.hasCompleted() is True:
                explorerThread.join()
                keepRunning = False

            
            
