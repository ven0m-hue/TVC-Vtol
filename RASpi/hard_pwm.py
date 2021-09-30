#!/usr/bin/env python
# coding: utf-8
import time
import pigpio
Set_GPIO = 18 # All gpios are identified as per BCM number.
pi = pigpio.pi() # Calling the pigpio class method pi()


# Given the angle convert it to duty cycle 
# dutycycle = ((angle/180.0) + 1.0) * 5
servo_f = 50
cycle_range = 20000


# Function to convert from angle to dutycycle 
def duty_cycle(angle):
    dutycycle = ((9.4/180) * angle + 2.95) # Hard coded, based on the calculation.
    # Side note -->Always try to linearize the slope of deg vs dutycycle curve.
    return dutycycle * 10**4
    

# Function to convert from dutycycle to pulse width 
def pulse_width(cycle):
    pulse_width = float(((1/ (servo_f)) * cycle))
    return pulse_width


pi.set_PWM_frequency(Set_GPIO, servo_f) # set the 50Hz servo 
pi.set_PWM_range(Set_GPIO, cycle_range) # set the servo PWM range 


try:
    
    while True:
        
        angle = float(input("Input the desired angle b/w 90 to 180 deg : "))
        pi.set_PWM_dutycycle(Set_GPIO, pulse_width(duty_cycle(angle))) #PWM off
        time.sleep(1) 
        
except KeyboardInterrupt:
        pass
        
print("Cleaning up..")
pi.set_PWM_dutycycle(Set_GPIO, 0)
pi.stop()







