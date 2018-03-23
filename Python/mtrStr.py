from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP2, eQEP1
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
from time import sleep

Enc1 = RotaryEncoder(eQEP1)
Enc1.enable()
Enc1.zero()
Enc1.setAbsolute()
Enc2 = RotaryEncoder(eQEP2)
Enc2.enable()
Enc2.zero()
Enc2.setAbsolute()

    
ADC.setup()	# setup BBB ADC
PWM1="P8_13"	# motor 1 signal
PWM2="P8_19"	# motor 2 signal
PWM.start(PWM1, 60)
PWM.start(PWM2, 60)
PWM.set_duty_cycle(PWM1, 69)
PWM.set_duty_cycle(PWM2, 71)

##### setup BBB GPIO pins
GPIO.setup("P8_37", GPIO.OUT)
GPIO.setup("P8_38", GPIO.OUT)
GPIO.setup("P8_31", GPIO.OUT)
GPIO.setup("P8_32", GPIO.OUT)

def back(slp):    ###BACK
    GPIO.output("P8_37", GPIO.LOW)
    GPIO.output("P8_38", GPIO.HIGH)
    GPIO.output("P8_31", GPIO.LOW)
    GPIO.output("P8_32", GPIO.HIGH)
    print("BACK")
    time.sleep(slp)

def right(slp):    ###RIGHT
    GPIO.output("P8_37", GPIO.HIGH)
    GPIO.output("P8_38", GPIO.LOW)
    GPIO.output("P8_31", GPIO.LOW)
    GPIO.output("P8_32", GPIO.HIGH)
    print("RIGHT")
    time.sleep(slp)

def forward(slp):    ###FORWARD
    GPIO.output("P8_37", GPIO.HIGH)
    GPIO.output("P8_38", GPIO.LOW)
    GPIO.output("P8_31", GPIO.HIGH)
    GPIO.output("P8_32", GPIO.LOW)
    print("FORWARD")
    time.sleep(slp)

def left(slp):    ###LEFT
    GPIO.output("P8_37", GPIO.LOW)
    GPIO.output("P8_38", GPIO.HIGH)
    GPIO.output("P8_31", GPIO.HIGH)
    GPIO.output("P8_32", GPIO.LOW)
    print("LEFT")
    time.sleep(slp)

def stop(slp):    ###STOP
    GPIO.output("P8_37", GPIO.LOW)
    GPIO.output("P8_38", GPIO.LOW)
    GPIO.output("P8_31", GPIO.LOW)
    GPIO.output("P8_32", GPIO.LOW)
    print("STOP")
    time.sleep(slp)

while True:
    stop(1)
    enc1 = Enc1.position/330
    enc2 = Enc2.position/330
    forward(3)
    print "Enc1: ", enc1, "Enc2: ", enc2
    back(3)
    print "Enc1: ", enc1, "Enc2: ", enc2
    right(3)
    print "Enc1: ", enc1, "Enc2: ", enc2
    left(3)
    print "Enc1: ", enc1, "Enc2: ", enc2
    
PWM.stop(PWM1)
PWM.stop(PWM2)
GPIO.cleanup()
PWM.cleanup()
