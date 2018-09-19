from multiprocessing import Pool;
from multiprocessing import Manager;
import child1;  #P2
import child23; #P3, 4
import child4;  #P17
import child5;  #P27
import child6;  #P22

def test():
	print(cur);

def check_formatting(data):
	if len(data) < 2: return False;
	return data[len(data) - 1].isnumeric() and data[len(data) - 2].isnumeric();

def stop(val):
    if val == 1:
        child1.stop(cur);
    elif val == 2 or val == 3:
        child23.stop(cur);
    elif val == 4:
        child4.stop(cur);
    elif val == 5:
        child5.stop(cur);
    elif val == 6:
        child6.stop(cur);
    return "child" + str(val) + " Stopped.";

def halt():
    print("");
    pool = Pool(processes = 4);
    results = pool.map(stop, (1, 2, 4, 5, 6));
    print(results);
    exit();

cur = [];
query = [-1, -1, -1, -1, -1, -1, -1];

def dequeue():
	print("dequeue process called");
	print(query);
	pool2 = Pool(processes = 4);
	results = pool2.starmap_async(update, [(1, query[1]), (2, query[2]), (3, query[3]), (4, query[4]), (5, query[5]), (6, query[6])]);
	pool2.close(); pool2.join();

def reset():
	for i in range(1, 7):
		query[i] = -1;
	print("query array reset");


def update(servo, val):
	if val == -1:
		print("no action performed on call " + str(servo) + ": " + str(val));
		return "stopped";
	if servo == 1:
		child1.moveto(servo, val, cur);
		print("Servo 1 moved to " + str(cur[1]));
	elif servo == 2:
		# child23.moveto(servo, val, cur);
		child23.s2(val, cur);
		print("Servo 2 moved to " + str(cur[2]) + ". Servo 3 moved to " + str(cur[3]));
	elif servo == 3:
		# child23.moveto(servo, val, cur);
		child23.s3(val, cur);
		print("Servo 2 moved to " + str(cur[2]) + ". Servo 3 moved to " + str(cur[3]));
	elif servo == 4:
		child4.moveto(servo, val, cur);
		print("Servo 4 moved to " + str(cur[4]));
	elif servo == 5:
		child5.moveto(servo, val, cur);
		print("Servo 5 moved to " + str(cur[5]));
	elif servo == 6:
		child6.moveto(servo, val, cur);
		print("Servo 6 moved to " + str(cur[6]));
	return "fin";

if __name__ == '__main__':
	manager = Manager();
	cur = manager.list([0, 1350, 2100, 625, 1400, 1400, 1400]);
	try:
		while True:
			try:
				data = input().split();
				if data[0] == "stop" or data[0] == ".":
					halt();
				elif data[0] == "update":
					dequeue();
					reset();
				elif data[0] == "test":
					test();
				elif data[0] == "debug":
					print(cur);
				elif check_formatting(data):
					try:
						servo = int(data[len(data) - 2]);
						val   = int(data[len(data) - 1]);

						if data[0] == "push":
							query[servo] = val;
							print("pushed " + str(val) + " to Servo " + str(servo));
						else:
							update(servo, val);
					except:
						print("Unable to parse data.");
			except KeyboardInterrupt:
				halt();
	except KeyboardInterrupt:
		halt();


halt();
exit();