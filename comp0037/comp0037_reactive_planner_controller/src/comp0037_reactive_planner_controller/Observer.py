import math
import threading


def shortestAngularDistance(self, fromAngle, toAngle):
    delta = toAngle - fromAngle
    if delta < -math.pi:
        delta = delta + 2.0 * math.pi
    elif delta > math.pi:
        delta = delta - 2.0 * math.pi
    return delta


def get_distance(controller, last_x, last_y):
    distance = math.sqrt(pow((last_x - controller.pose.x), 2) + pow((last_y - controller.pose.y), 2))
    return distance


def MyThread1(controller):
    last_x = controller.pose.x
    last_y = controller.pose.y

    # while True:



def runMyThread(controller):
    t1 = threading.Thread(target=MyThread1, args=[controller])
    t1.start()
