import pigpio;
import time;

pi = pigpio.pi();

PIN4 = 17;

pi.set_mode(PIN4, pigpio.OUTPUT);

wait = 0.008;


pi.set_servo_pulsewidth(PIN4, 1400);

def moveto(num, end, cur):
	if num == 4:
		if end >= 500 and end <= 2135:
			s4(end, cur);
		else:
			print("Range4: 500->2135");

def s4(val, cur):
	if cur[4] < val:
		while cur[4] < val:
			cur[4] += 5;
			pi.set_servo_pulsewidth(PIN4, cur[4]);
			time.sleep(wait);
	elif cur[4] > val:
		while cur[4] > val:
			cur[4] -= 5;
			pi.set_servo_pulsewidth(PIN4, cur[4]);
			time.sleep(wait);

def stop(cur):
	s4(1400, cur);
	time.sleep(0.2);
	pi.set_servo_pulsewidth(PIN4, 0);
	pi.stop();
	print("4 Stopped.");