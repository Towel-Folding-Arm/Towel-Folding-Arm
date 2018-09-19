import pigpio;
import time;
from multiprocessing import Pool;

pi = pigpio.pi();

PIN2 = 3;
PIN3 = 4;

pi.set_mode(PIN2, pigpio.OUTPUT);
pi.set_mode(PIN3, pigpio.OUTPUT);

wait = 0.008;

pi.set_servo_pulsewidth(PIN2, 2100);
pi.set_servo_pulsewidth(PIN3, 625);

def moveto(num, end, cur):
    if num == 2:
        if end >= 850 and end <= 2100:
            if end < cur[2]: #away
                away(end, cur[3], cur);
            elif end > cur[2]: #back
                end3 = cur[3];
                if 2100 - end < cur[3] - 625:
                    end3 = 2100 - end + 625;
                back(end, end3, cur);
        else:
            print("Range2: 2100->850");
    elif num == 3:
        if end >= 625 and end <= 1700:
            if end > cur[3]: #away
                end2 = cur[2];
                if end - 625 > 2100 - cur[2]:
                    end2 = 2100 - (end - 625);
                away(end2, end, cur);
            elif end < cur[3]: #back
                back(cur[2], end, cur);
        else:
            print("Range3: 625->1700");

def s2(val, cur):
    if cur[2] < val:
        while cur[2] < val:
            cur[2] += 5;
            pi.set_servo_pulsewidth(PIN2, cur[2]);
            time.sleep(wait);
    elif cur[2] > val:
        while cur[2] > val:
            cur[2] -= 5;
            pi.set_servo_pulsewidth(PIN2, cur[2]);
            time.sleep(wait);

def s3(val, cur):
    if cur[3] < val:
        while cur[3] < val:
            cur[3] += 5;
            pi.set_servo_pulsewidth(PIN3, cur[3]);
            time.sleep(wait);
    elif cur[3] > val:
        while cur[3] > val:
            cur[3] -= 5;
            pi.set_servo_pulsewidth(PIN3, cur[3]);
            time.sleep(wait);

def multi(servo, val2, val3, cur):
    if servo == 2:
        s2(val2, cur);
    elif servo == 3:
        s3(val3, cur);

def back(val2, val3, cur):
    pool3 = Pool(processes = 2);
    results = pool3.starmap_async(multi, [(2, val2, val3, cur), (3, val2, val3, cur)]);
    pool3.close(); pool3.join();
#    while cur[3] > val3:
#        cur[3] -= 5;
#        pi.set_servo_pulsewidth(PIN3, cur[3]);
#        time.sleep(wait);
#    while cur[2] < val2 - 5:
#        cur[2] += 5;
#        pi.set_servo_pulsewidth(PIN2, cur[2]);
#        time.sleep(wait);

def away(val2, val3, cur):
    pool4 = Pool(processes = 2);
    results = pool4.starmap_async(multi, [(2, val2, val3, cur), (3, val2, val3, cur)]);
    pool4.close(); pool4.join();
#    while cur[2] > val2:
#        cur[2] -= 5;
#        pi.set_servo_pulsewidth(PIN2, cur[2]);
#        time.sleep(wait);
#    while cur[3] < val3 - 5:
#        cur[3] += 5;
#        pi.set_servo_pulsewidth(PIN3, cur[3]);
#        time.sleep(wait);
#

def stop(cur):
    # back(2100, 625, cur);
    s3(625, cur);
    s2(2100, cur);
    time.sleep(0.2);
    pi.set_servo_pulsewidth(PIN3, 0);
    pi.set_servo_pulsewidth(PIN2, 0);
    pi.stop();
    print("2 3 Stopped.");
    #exit();

