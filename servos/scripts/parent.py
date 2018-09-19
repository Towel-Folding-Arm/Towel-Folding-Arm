#!/usr/bin/env python
import rospy
import re
import time
from std_msgs.msg import String
from multiprocessing import Pool
from multiprocessing import Manager
import child1  #Pin 2
import child23 #Pins 3, 4
import child4  #Pin 17
import child5  #Pin 27
import child6  #Pin 22
import gripper #Pins 14, 15, 18

cur = []
query = [-1, -1, -1, -1, -1, -1, -1]
delp  = [-1, -1, -1, -1, -1, -1, -1]
wait  = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
start = []

def update(servo):
	if query[servo] == -1:
		print("no action performed on call " + str(servo) + ": " + str(query[servo]))
		return "stopped"
	if servo == 1:
		child1.moveto(servo, query[servo], cur, wait[1])
		print(time.time() - start[0], "\tServo 1 moved to " + str(cur[1]))
	elif servo == 2:
		# child23.moveto(servo, val, cur)
		child23.s2(query[servo], cur, wait[2])
		print(time.time() - start[0], "\tServo 2 moved to " + str(cur[2]) + ". Servo 3 moved to " + str(cur[3]))
	elif servo == 3:
		# child23.moveto(servo, val, cur)
		child23.s3(query[servo], cur, wait[3])
		print(time.time() - start[0], "\tServo 2 moved to " + str(cur[2]) + ". Servo 3 moved to " + str(cur[3]))
	elif servo == 4:
		child4.moveto(servo, query[servo], cur, wait[4])
		print(time.time() - start[0], "\tServo 4 moved to " + str(cur[4]))
	elif servo == 5:
		child5.moveto(servo, query[servo], cur, wait[5])
		print(time.time() - start[0], "\tServo 5 moved to " + str(cur[5]))
	elif servo == 6:
		child6.moveto(servo, query[servo], cur, wait[6])
		print(time.time() - start[0], "\tServo 6 moved to " + str(cur[6]))
	return "fin"

def dequeue():
	print("dequeue process called")
	print(query)

	for i in range(1, 7):
		delp[i] = abs(cur[i] - query[i])
		if delp[i] < 1:
			delp[i] = 0
		if (delp[i] != 0):
			wait[i] = 1.0 / delp[i]
		else:
			wait[i] = 0.001

	print(wait)
	start[0] = time.time()
	pool2 = Pool(processes = 6)
	results = pool2.map(update, [1, 2, 3, 4, 5, 6])
	pool2.close()
	pool2.join()
	print(time.time() - start[0])
	print("dequeue process finished")

def callback(data):
	s = str(data)
	print(s)
	data = re.split('data: "| "| |"', s)
	print(len(data))
	print(data)
	if len(data) == 3:
		if data[1] == 'o':
			gripper.open()
		elif data[1] == 'c':
			gripper.close()
		else:
			gripper.stop()
	elif len(data) == 8:
		for i in range(1, 7):
			query[i] = float(data[i])
		dequeue()
	

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("PWMvalues", String, callback)
	rospy.spin()

if __name__ == '__main__':
	manager = Manager()
	cur = manager.list([0, 1350, 2100, 625, 1400, 1500, 1150])
	start = manager.list([0.0])
	listener()

def stop(val):
    if val == 1:
        child1.stop(cur)
    elif val == 2 or val == 3:
        child23.stop(cur)
    elif val == 4:
        child4.stop(cur)
    elif val == 5:
        child5.stop(cur)
    elif val == 6:
        child6.stop(cur)
    return "child" + str(val) + " Stopped."

def halt():
    print("")
    gripper.stop()
    pool = Pool(processes = 6)
    results = pool.map(stop, (1, 2, 4, 5, 6))
    print(results)
    exit()

halt()
exit()
