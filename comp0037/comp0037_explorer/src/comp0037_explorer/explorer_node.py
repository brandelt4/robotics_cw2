import rospy

from explorer_node_base import ExplorerNodeBase

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []


    def chooseNewDestination(self):


#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)


        # Iterates through coordinates X and Y

        for candidate in self.frontier:
            print(candidate)
            candidateGood = True
            for k in range(0, len(self.blackList)):
                if self.blackList[k] == candidate:
                    candidateGood = False
                    break

            if candidateGood is True:
                return True, candidate

        return False, None

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
