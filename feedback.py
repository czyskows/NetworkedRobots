
import socket
from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP2, eQEP1
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
#import numpy as np
import magnet
import math
import threading
import keys

ADC.setup()# setup BBB ADC
PWM1="P8_13"# motor 1 signal
PWM2="P8_19"# motor 2 signal
##### setup BBB GPIO pins
GPIO.setup("P8_37", GPIO.OUT)
GPIO.setup("P8_38", GPIO.OUT)
GPIO.setup("P8_31", GPIO.OUT)
GPIO.setup("P8_32", GPIO.OUT)
PWM.start(PWM1, 60)
PWM.start(PWM2, 60)

Enc1 = RotaryEncoder(eQEP1)
Enc1.enable()
if(Enc1.enabled == 1):
    print("Enc1 Enabled")
else:
    print("Enc1 NOT enabled")
Enc1.setAbsolute()
Enc1.zero()
Enc2 = RotaryEncoder(eQEP2)
Enc2.enable()
if(Enc2.enabled == 1):
    print("Enc2 Enabled")
else:
    print("Enc2 NOT enabled")
    Enc2.setAbsolute()
    Enc2.zero()

def get_arrows():
    key = keys.read_key()
    if key == ARROW_LEFT:
        angVel -= 10
    elif key == ARROW_RIGHT:
        angVel += 10
    elif key == ARROW_UP:
        p1 = p1 + 5
        calcDuty(duty, p1, p2)
    elif key == ARROW_DOWN:
        p1 = p1 - 5
        calcDuty(duty, p1, p2)
    elif key == SPACE:
        setDuty(0, PWM1, dir1, dir2)
        setDuty(0, PWM2, dir1, dir2)
        
def scale(val, inMin, inMax, outMin, outMax):
    inSpan = inMax - inMin
    outSpan = outMax - outMin
    valScaled = float(val - inMin) / float(inSpan)
    return outMin + (valScaled * outSpan)

def forward(dutyL, dutyR):
    PWM.set_duty_cycle(PWM1, dutyL)
    PWM.set_duty_cycle(PWM2, dutyR) 
    GPIO.output("P8_37", GPIO.HIGH)
    GPIO.output("P8_38", GPIO.LOW)
    GPIO.output("P8_31", GPIO.HIGH)
    GPIO.output("P8_32", GPIO.LOW)

def calcDuty(duty, p1, p2):
    if duty > 100:
        duty = 100
    elif duty > p2:
        duty = p1 + (1-p1/100)*duty
    elif duty > -p2:
        duty = (1+(p1/p2)-(p1/100))*duty
    elif duty > -100:
        duty = -p1 + (1-p1/100)*duty
    else:
        duty = -100
    return duty

def setDuty(duty, pwm, dir1, dir2):
    if duty > 0:
        PWM.set_duty_cycle(pwm, duty)
        GPIO.output(dir1, GPIO.HIGH)
        GPIO.output(dir2, GPIO.LOW)
    elif duty < 0:
        PWM.set_duty_cycle(pwm, -duty)
        GPIO.output(dir1, GPIO.LOW)
        GPIO.output(dir2, GPIO.HIGH)
            
        
K = 1
D = 18 # centimeters = tread distance
R2Pi = 3*2*math.pi # cm
KVel = 20 # forward velocity
KAng = 10 # angular velocity
forwardVelRef = 1 # cm per second
angVelRef = 0 # radians per second
dt = .1 # sleep time
Lmda = .01 # filter value
p1 = 70 # duty cycle multiplier
p2 = 1                   
print("newTime\t\tforVel\tu1\tangVel\tu2\tvel1\tPWMLeft\tp1\tvel2\tPWMRight\tp2")

# init values
oldEnc1 = Enc1.position/333
oldEnc2 = Enc2.position/333
oldTime = time.perf_counter()
oldVel1 = 0
oldVel2 = 0
time.sleep(dt)

try:

    while True:

        # read encoders   
        enc1 = Enc1.position/333
        enc2 = Enc2.position/333

        # get time
        newTime = time.perf_counter()

        # calculate velocities
        newVel1 = R2Pi*(enc1 - oldEnc1)/(newTime-oldTime) #cm per second
        newVel2 = R2Pi*(enc2 - oldEnc2)/(newTime-oldTime) #cm per second

        # low-pass filter
        vel1 = newVel1*Lmda + oldVel1*(1-Lmda)
        vel2 = newVel2*Lmda + oldVel2*(1-Lmda)
        
        forwardVel = (vel1+vel2)/2
        angVel = (vel1-vel2)/D # radians per second

        # forward velocity feedback
        velError = forwardVelRef - forwardVel
        u1 = KVel*velError

        # calculate angular velocity feedback
        angVelError = angVelRef - angVel
        u2 = KAng*angVelError

        # calculate PWM
        
        PWMRight = (u1 + u2)/2
        newDutyR = calcDuty(PWMRight, p1, p2)
        PWMLeft = (u1 - u2)/2
        newDutyL = calcDuty(PWMLeft, p1, p2)
        setDuty(newDutyR, PWM1, "P8_37", "P8_38")
        setDuty(newDutyL, PWM2, "P8_31", "P8_32")

        print("{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}\t{:6.3f}".format(newTime,forwardVel,u1,angVel,u2,vel1,PWMLeft,newDutyL,vel2,PWMRight,newDutyR),end="\r")

        # get key values        
        thread = threading.Thread(target = get_arrows)
        thread.daemon = False
        thread.start()

        # update variables
        oldTime = newTime 
        oldEnc1 = enc1
        oldEnc2 = enc2

        # sleep for a while
        time.sleep(dt)

except KeyboardInterrupt:
    pass

finally:
    
    ###STOP
    GPIO.output("P8_37", GPIO.LOW)
    GPIO.output("P8_38", GPIO.LOW)
    GPIO.output("P8_31", GPIO.LOW)
    GPIO.output("P8_32", GPIO.LOW)
    PWM.start(PWM1, 60)
    PWM.start(PWM2, 60)
    PWM.set_duty_cycle(PWM1, 60)
    PWM.set_duty_cycle(PWM2, 60)
    
    PWM.stop(PWM1)
    PWM.stop(PWM2)
    GPIO.cleanup()
    PWM.cleanup()



    

