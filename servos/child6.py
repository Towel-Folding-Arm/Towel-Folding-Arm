import pigpio
import time

pi = pigpio.pi()

PIN6 = 22

pi.set_mode(PIN6, pigpio.OUTPUT)

pi.set_servo_pulsewidth(PIN6, 1150)

def moveto(num, end, cur, wait):
	if num == 6:
		if end >= 500 and end <= 1800:
			s6(end, cur, wait)
		else:
			print("Range6: 500->1800")

def s6(val, cur, wait):
	if cur[6] < val:
		while cur[6] < val:
			cur[6] += 1
			pi.set_servo_pulsewidth(PIN6, cur[6])
			time.sleep(wait)
	elif cur[6] > val:
		while cur[6] > val:
			cur[6] -= 1
			pi.set_servo_pulsewidth(PIN6, cur[6])
			time.sleep(wait)

def stop(cur):
	s6(1150, cur, 0.001)
	time.sleep(0.2)
	pi.set_servo_pulsewidth(PIN6, 0)
	pi.stop()
	print("6 Stopped.")
