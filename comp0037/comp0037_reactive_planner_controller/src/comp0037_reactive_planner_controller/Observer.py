import math
import threading
import time


def shortestAngularDistance(fromAngle, toAngle):
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
    last_theta = controller.pose.theta

    start = time.time()  # START
    total_distance = 0.0
    total_angle = 0.0
    print("crazy90\n"*90)
    while True:
        total_angle = total_angle + shortestAngularDistance(controller.pose.theta, last_theta)
        total_distance = total_distance + get_distance(controller, last_x, last_y)

        last_x = controller.pose.x
        last_y = controller.pose.y
        last_theta = controller.pose.theta

        f = open('../recorded_data.csv', "a")
        f.write(str(controller.goal).format())
        f.write("{},".format(str(time.time() - start)))
        f.write("{},".format(str(total_distance)))
        f.write("{},".format(str(total_angle * 360.0 / 6.28)))
        f.close()

	print(total_distance)

        time.sleep(1)


def runMyThread(controller):
    t1 = threading.Thread(target=MyThread1, args=[controller])
    t1.start()
