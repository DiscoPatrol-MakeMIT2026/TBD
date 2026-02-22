import pigpio, time

ESC = 18  # GPIO18

pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC, 0)
time.sleep(1)

print("Arming...")
pi.set_servo_pulsewidth(ESC, 2000)  # high
time.sleep(2)
pi.set_servo_pulsewidth(ESC, 1000)  # low
time.sleep(2)

print("Start low throttle")
pi.set_servo_pulsewidth(ESC, 1200)
time.sleep(5)

print("Stop")
pi.set_servo_pulsewidth(ESC, 0)