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
    while True:
        if controller.allow:
            break
        time.sleep(0.2)

    # print('RUNNING *'*40)
    last_x = controller.pose.x
    last_y = controller.pose.y
    last_theta = controller.pose.theta


    start = time.time()  # START
    total_distance = 0.0
    total_angle = 0.0

    loop_counter = 0
    while True:
        total_angle = total_angle + abs(shortestAngularDistance(controller.pose.theta, last_theta))
        total_distance = total_distance + get_distance(controller, last_x, last_y)

        last_x = controller.pose.x
        last_y = controller.pose.y
        last_theta = controller.pose.theta

        if loop_counter % 10 == 0:
            f = open('../recorded_data.csv', "a+")
            f.write("Time: {},".format(str(time.time() - start)))
            f.write("Distance: {},".format(str(total_distance)))
            f.write("Angle: {}\n\n".format(str(total_angle * 360.0 / 6.28)))
            f.close()
            loop_counter = 0

        loop_counter += 1
        time.sleep(0.1)




def runMyThread(controller):
    t1 = threading.Thread(target=MyThread1, args=[controller])
    t1.start()
