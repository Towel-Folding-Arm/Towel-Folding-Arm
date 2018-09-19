import pigpio;

pi = pigpio.pi();

PIN1 = 2;
PIN2 = 3;
PIN3 = 4;
PIN4 = 17;
PIN5 = 27;
PIN6 = 22;

pi.set_mode(PIN1, pigpio.OUTPUT);
pi.set_mode(PIN2, pigpio.OUTPUT);
pi.set_mode(PIN3, pigpio.OUTPUT);
pi.set_mode(PIN4, pigpio.OUTPUT);
pi.set_mode(PIN5, pigpio.OUTPUT);
pi.set_mode(PIN6, pigpio.OUTPUT);

pi.set_servo_pulsewidth(PIN1, 0);
pi.set_servo_pulsewidth(PIN2, 0);
pi.set_servo_pulsewidth(PIN3, 0);
pi.set_servo_pulsewidth(PIN4, 0);
pi.set_servo_pulsewidth(PIN5, 0);
pi.set_servo_pulsewidth(PIN6, 0);

pi.stop();
exit();