import pigpio
import time

pi = pigpio.pi()

PIN5 = 27

pi.set_mode(PIN5, pigpio.OUTPUT)

pi.set_servo_pulsewidth(PIN5, 1500)

def moveto(num, end, cur, wait):
	if num == 5:
		if end >= 650 and end <= 2270:
			s5(end, cur, wait)
		else:
			print("Range5: 650->2270")

def s5(val, cur, wait):
	if cur[5] < val:
		while cur[5] < val:
			cur[5] += 1
			pi.set_servo_pulsewidth(PIN5, cur[5])
			time.sleep(wait)
	elif cur[5] > val:
		while cur[5] > val:
			cur[5] -= 1
			pi.set_servo_pulsewidth(PIN5, cur[5])
			time.sleep(wait)

def stop(cur):
	s5(1500, cur, 0.001)
	time.sleep(0.2)
	pi.set_servo_pulsewidth(PIN5, 0)
	pi.stop()
	print("5 Stopped.")
