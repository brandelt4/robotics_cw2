import sys
sys.path.append('/home/ros_user/catkin_ws/src/comp0037/comp0037_reactive_planner_controller/src/comp0037_reactive_planner_controller')
sys.path.append('/home/ros_user/catkin_ws/src/comp0037/comp0037_mapper/src/comp0037_mapper')

from geometry_msgs.msg  import Pose2D
import rospy
from math import sqrt
from mapper_node import MapperNode
from explorer_node_base import ExplorerNodeBase
from reactive_planner_controller import ReactivePlannerController
import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg  import Pose2D


# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.selection = 'closest' # 'farthest', 'entropy'
        self.previousDestination = (0,0)
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
            # pose = Pose2D()
            # start = (pose.x, pose.y)
            # print(pose.x, pose.y)

            with open('/home/ros_user/catkin_ws/src/comp0037/comp0037_explorer/src/comp0037_explorer/position.txt', 'r') as file:
                try:
                    positionX = float(file.readline())
                    positionY = float(file.readline())
                    start = (positionX, positionY)
                except:
                    pass

            startCellCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(start)
            print("Start cell coordinates: {}".format(startCellCoords))
            for idx, candidate in enumerate(self.frontier):
                candidateGood = True
                costs.append(self.calculateHeuristic(startCellCoords, candidate))

            # CHOOSES THE CLOSEST CELL
            if self.selection == 'closest':
                nextOne = costs.index(min(costs))
                nextOne_blacklist = []
                # Check if in blacklist
                while self.frontier[nextOne] in self.blackList or self.frontier[nextOne] == startCellCoords:
                    # List of those indeces that you should not choose
                    nextOne_blacklist.append(nextOne)
                    if nextOne == len(self.frontier):
                        return False, None
                    else:
                        # Find the next smallest number
                        nextSmallest = max(costs)
                        i = 0
                        print(costs)
                        for cost in costs:
                            # If the cost is smaller than the current smallest and the index is not the same
                            if (cost < nextSmallest) and not (i in nextOne_blacklist):
                                nextSmallest = cost
                            i += 1
                        # Find index of the next smallest
                        nextOne = costs.index(nextSmallest)
                        print(nextOne)


                print("Next cell coordinates: {}".format(self.frontier[nextOne]))

            if (candidateGood is True) and (self.frontier[nextOne] != self.previousDestination) and (self.frontier[nextOne] != startCellCoords):
                return True, self.frontier[nextOne]

        return False, None

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
