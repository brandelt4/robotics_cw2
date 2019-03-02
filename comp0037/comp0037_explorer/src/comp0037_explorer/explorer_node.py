import sys
sys.path.append('/home/ros_user/catkin_ws/src/comp0037/comp0037_reactive_planner_controller/src/comp0037_reactive_planner_controller')
sys.path.append('/home/ros_user/catkin_ws/src/comp0037/comp0037_mapper/src/comp0037_mapper')


import rospy
from math import sqrt
from mapper_node import MapperNode
from explorer_node_base import ExplorerNodeBase
from reactive_planner_controller import ReactivePlannerController
import copy

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []

    def calculateHeuristic(self, start, goal):
        dX = start[0] - goal[0]
        dY = start[1] - goal[1]
        return sqrt(dX ** 2 + dY ** 2)

    def chooseNewDestination(self):


#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        costs = []
        candidateGood = False

        # Iterates through coordinates X and Y

        # NOW IMPLEMENT THE BEST WAY TO CHOOSE THE NEW DESTINATION

        if self.counter == 1:
            return True, self.frontier[0]

        else:
            pose = ReactivePlannerController.controller.getCurrentPose()
            start = (pose.x, pose.y)
            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
            for idx, candidate in enumerate(self.frontier):
                candidateGood = True
                costs.append(self.calculateHeuristic(startCellCoords, candidate))

            nextOne = costs.index(min(costs))

            if candidateGood is True:
                return True, self.frontier[nextOne]



        return False, None

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
