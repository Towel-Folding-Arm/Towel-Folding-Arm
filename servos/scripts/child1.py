import pigpio
import time

pi = pigpio.pi()

PIN1 = 2

pi.set_mode(PIN1, pigpio.OUTPUT)

pi.set_servo_pulsewidth(PIN1, 1350)

def moveto(num, end, cur, wait):
	if num == 1:
		if end >= 550 and end <= 2100:
			s1(end, cur, wait)
		else:
			print("Range1: 550->2100")

def s1(val, cur, wait):
	if cur[1] < val:
		while cur[1] < val:
			cur[1] += 1
			pi.set_servo_pulsewidth(PIN1, cur[1])
			time.sleep(wait)
	elif cur[1] > val:
		while cur[1] > val:
			cur[1] -= 1
			pi.set_servo_pulsewidth(PIN1, cur[1])
			time.sleep(wait)

def stop(cur):
	s1(1350, cur, 0.001)
	time.sleep(0.2)
	pi.set_servo_pulsewidth(PIN1, 0)
	pi.stop()
	print("1 Stopped.")
