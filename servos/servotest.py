import pigpio;
import time;

pi = pigpio.pi();

PIN = 2;

pi.set_mode(PIN, pigpio.OUTPUT);

wait = 0.008;
cur = 1500;

pi.set_servo_pulsewidth(PIN, cur);

def moveto(cur, val):
	if cur < val:
		while cur < val:
			cur += 5;
			pi.set_servo_pulsewidth(PIN, cur);
			time.sleep(wait);
	elif cur > val:
		while cur > val:
			cur -= 5;
			pi.set_servo_pulsewidth(PIN, cur);
			time.sleep(wait);
	return val;

def stop():
	moveto(cur, 1500);
	time.sleep(0.2);
	pi.set_servo_pulsewidth(PIN, 0);
	pi.stop();
	print("Stopped.");
	exit();

try:
	while True:
		try:
			data = raw_input();
			if data == "stop":
				stop();
			else:
				try:
					val = float(data);
					print("parsed");
					cur = moveto(cur, val);
					print("moved");
					print("cur " + str(cur));
				except:
					print("parsing error");
		except KeyboardInterrupt:
			stop();
except KeyboardInterrupt:
	stop();

stop();