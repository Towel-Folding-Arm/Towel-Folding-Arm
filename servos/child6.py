import pigpio;
import time;

pi = pigpio.pi();

PIN6 = 22;

pi.set_mode(PIN6, pigpio.OUTPUT);

wait = 0.008;

pi.set_servo_pulsewidth(PIN6, 1400);

def moveto(num, end, cur):
	if num == 6:
		if end >= 500 and end <= 2300:
			s6(end, cur);
		else:
			print("Range6: 500->2300");

def s6(val, cur):
	if cur[6] < val:
		while cur[6] < val:
			cur[6] += 5;
			pi.set_servo_pulsewidth(PIN6, cur[6]);
			time.sleep(wait);
	elif cur[6] > val:
		while cur[6] > val:
			cur[6] -= 5;
			pi.set_servo_pulsewidth(PIN6, cur[6]);
			time.sleep(wait);

def stop(cur):
	s6(1400, cur);
	time.sleep(0.2);
	pi.set_servo_pulsewidth(PIN6, 0);
	pi.stop();
	print("6 Stopped.");