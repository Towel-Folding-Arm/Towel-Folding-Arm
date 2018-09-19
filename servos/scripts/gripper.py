import pigpio
import time

pi = pigpio.pi()

enablePin = 14
motorPin1 = 15
motorPin2 = 18

pi.set_mode(enablePin, pigpio.OUTPUT)
pi.set_mode(motorPin1, pigpio.OUTPUT)
pi.set_mode(motorPin2, pigpio.OUTPUT)

def open():
	pi.write(motorPin1, 0)
	pi.write(motorPin2, 1)
	time.sleep(1)
	stop()

def close():
	pi.write(motorPin1, 1)
	pi.write(motorPin2, 0)

def stop():
	pi.write(motorPin1, 0)
	pi.write(motorPin2, 0)

pi.write(enablePin, 1)
close()
time.sleep(3.5);
open()
open()
stop()

