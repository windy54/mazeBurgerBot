import gc
import time
from motor import Motor, pico_motor_shim
from encoder import Encoder, MMME_CPR
from pimoroni import Button, PID, NORMAL_DIR   , REVERSED_DIR
#range sensors
from hcsr04 import HCSR04
import _thread
from machine import Pin
"""
mazeBurgerBot.py
attempts to solve PI Wars Maze challenge

this version has been tidied up to use functions amd the Pimoroni encoder library
I have also removed the PID control and just a straight gain.
Probably gone overboard with defining variables as  global because I get the occassional
variable not defined error, e.g. sonars
At the moment, if the robot is left moving forward for (say) 20 seconds the right motor
stops. Then if I put my hand in front (Robot on a test stand) to simulare reaching a
wall, the code steps to making a turn and the left motor stops.
"""
def readEncoders():
    # Print out the angle of each encoder
    global encleft, encRight
    print("encLeft =", encLeft.count(), end=", ")
    print("encRight =", encRight.count(), end=", ")
    print()

def readAllSonars():
    global sonarValues, sonars, distanceLock
    localValues = [-10, -10, -10]
    while True:
        for index in range(3):
            localValues[index] = sonars[index].distance_cm()
        distanceLock.acquire()
        sonarValues = localValues
        distanceLock.release()
        time.sleep_us(1000 ) #(500000)

def getNewRange():
    global distanceLock, sonarValues
    
    distanceLock.acquire()
    newValues = sonarValues
    distanceLock.release()
    return newValues

def legForwards():
        global legMode, mode, moveLeftMode, moveRightMode, forwardsMode, stopMode
        global leftDemand, rightDemand 
        global minDistance, frontDistance, leftDistance, rightDistance,edgeLimit
        global iterationCount, startLeftRightTurn, turnDelay
        global leftrevspersec,rightrevspersec
        global led, encLeft, encRight, mLeft, mRight, leftVel, rightVel
        # mode logic
        #print(mode, frontDistance)
        led.off()
        if minDistance < frontDistance < 250  : # safe to move forward
            #print("safe to move forward {0}".format(frontDistance) )
            if mode == moveLeftMode or mode == moveRightMode :
                #print("firstError {0} latest Error {1}".format(firstError, error))
                #if error * firstError < 0: # sign change so back on centreline
                if iterationCount - startLeftRightTurn > turnDelay:
                    mode = forwardsMode
                    leftVel  = leftrevspersec
                    rightVel  = rightrevspersec
                    #print("left speed {0}  right speed {1}".format(leftVel,rightVel))
            elif edgeLimit > leftDistance : # in side left limit so move right
                mode = moveRightMode
                startLeftRightTurn = iterationCount
                turnDelay = 0
                leftVel  = leftrevspersec
                rightVel = turnrevspersec
                #print("left speed {0}  right speed {1}".format(leftVel,rightVel))
                #print("firstError {0}".format(firstError))
            elif rightDistance < edgeLimit:
                mode = moveLeftMode
                leftVel  = turnrevspersec
                rightVel = rightrevspersec
                startLeftRightTurn = iterationCount
                turnDelay = 1
                #print("left speed {0}  right speed {1}".format(leftVel,rightVel))
                #print("firstError {0}".format(firstError))
            elif mode == forwardsMode : # reset to forwards mode to aid testing
                leftVel  = leftrevspersec
                rightVel = rightrevspersec
                #print("left speed {0}  right speed {1}".format(leftVel,rightVel))
            else : # mode must be stop
                mode = forwardsMode
                leftVel= leftrevspersec
                rightVel = rightrevspersec
        elif frontDistance < minDistance : # too close to front wall so turn
            legMode+=1 # need general logic to go onto next leg, turnright and left can be achieved by changing setpoint
            mode = stopMode
            leftVel = 0
            rightVel = 0
            encLeft.zero()
            encRight.zero()
            count4Ninetydegs = 700
            if legMode > 4:
                leftDemand = -count4Ninetydegs
                rightDemand = count4Ninetydegs
            else:
                leftDemand = count4Ninetydegs
                rightDemand = -count4Ninetydegs
            print("start turn {0}".format(legMode))

        else: # stop invalid reading
            leftVel = 0
            rightVel = 0
            led.on()
            
            
        mLeft.speed(leftVel)
        mRight.speed(rightVel)

def legTurn(): # direction specified by setting setpoint
    global leftDemand, rightDemand, legMode,  mLeft, mRight, encLeft, encRight
    
    def calcrateDemand(position, demand):
        error = (demand - position )
        return error * 0.025
    
    lPos = encLeft.count()
    leftVel = calcrateDemand(lPos , leftDemand)
    mLeft.speed(leftVel)
    
    rPos = encRight.count()
    rightVel = calcrateDemand(rPos , rightDemand)
    mRight.speed(rightVel)
    
    # add logic to detect when turn complete monitor error ?
    x = lPos - leftDemand
    y = rPos - rightDemand
    error = (x)**2 + (y)**2
    #print(error, leftDemand,rightDemand, lPos, rPos )
    if  error < 10: # look for errors < 2 counts in both axes
        legMode+=1
        leftVel = 0
        rightVel = 0
        mLeft.speed(leftVel)
        mRight.speed(rightVel)
        encLeft.zero()
        encRight.zero()
        print("Turn complete ", lPos, rPos)
        time.sleep(1)
def stop():
    mLeft.speed(0)
    mRight.speed(0)
def finished():
    global legMode, letsGo
    legMode+=1
    letsGo = False
    print("finished")

# Free up hardware resources ahead of creating a new Encoder
gc.collect()

MOTOR_PINS_RIGHT = pico_motor_shim.MOTOR_1          # The pins of the motor being profiled
MOTOR_PINS_LEFT = pico_motor_shim.MOTOR_2          # The pins of the motor being profiled
ENCODER_PINS_RIGHT = (16,17) #(17, 16)      # The pins of the encoder attached to the profiled motor
ENCODER_PINS_LEFT = (15,14) #(17, 16)      # The pins of the encoder attached to the profiled motor
GEAR_RATIO = 100                         # The gear ratio of the motor
COUNTS_PER_REV = 28 * GEAR_RATIO  # The counts per revolution of the motor's output shaft
DIRECTION = NORMAL_DIR
SPEED_SCALE = 1.0
# The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
# Create an encoder, using PIO 0 and State Machine 0
# Create an encoder, using PIO 0 and State Machine 0
encLeft  = Encoder(0, 0, ENCODER_PINS_LEFT, direction=DIRECTION, counts_per_rev=COUNTS_PER_REV, count_microsteps=True)
encRight = Encoder(1, 0, ENCODER_PINS_RIGHT, direction=REVERSED_DIR, counts_per_rev=COUNTS_PER_REV, count_microsteps=True)
# Create a motor and set its speed scale
mLeft  = Motor(MOTOR_PINS_LEFT, direction=REVERSED_DIR, speed_scale=SPEED_SCALE)
mRight = Motor(MOTOR_PINS_RIGHT, direction=REVERSED_DIR, speed_scale=SPEED_SCALE)
mRight = Motor(MOTOR_PINS_RIGHT, direction=REVERSED_DIR, speed_scale=SPEED_SCALE)

# create led
led = Pin(25, Pin.OUT)
# Create the user button

user_sw = Button(pico_motor_shim.BUTTON_A)
# Enable the motor to get started
mLeft.enable()
mRight.enable()

revspersec = 0.9 # for test 1
turnrevspersec = 0.6
'''
speed     offset
0.5        0.001
1.0
0.8        0.01
0.9        0.014 not quite enough 0.013?
o = m*speed +c
0.001 = m * 0.5 +c
0.01 = m * 0.8 +c
0.3m = 0.009 => m= 0.03
c = 0.001 - 0.5*0.03 => c = -0.014
so for m= 1
offset = 0.03*1--0.014 = 0.044
'''
# speed defintions
offsetrevspersec = 0.014  #speed dependant - offset to move in straight linr
leftrevspersec = revspersec + offsetrevspersec   
rightrevspersec = revspersec
leftVel = 0
rightVel= 0
#######################################################################
# sonar
# a timeout of 10000 micro secs is 10msecs or 100Hz
# so worst case could take 30 msecs to read all sensors

frontSonar = HCSR04(trigger_pin=18, echo_pin=20, echo_timeout_us=10000)
leftSensor = HCSR04(trigger_pin=10, echo_pin=11, echo_timeout_us=10000)
rightSensor = HCSR04(trigger_pin=13, echo_pin=12, echo_timeout_us=10000)

sonarValues = [-10, -10, -10] # initialise to invalid values [0] is left [1] is forward [2] is right
sonars = [leftSensor, frontSonar , rightSensor]
distanceLock = _thread.allocate_lock()
#######################################################################
_thread.start_new_thread(readAllSonars, () )
# mode defintion within a Legmode
stopMode = 0
forwardsMode = 1
moveLeftMode = 2
moveRightMode = 3
mode = forwardsMode
modeText = ["stop","forwards","left","right"]
lastmode = -1
legModeText = ["Forwards", "Right", "Forwards", "Right", "Forwards", "Left", "Forwards", "Left", "Forwards", "Stop"]
legMode = 0 # Forwards
caseStatement = {
    0: legForwards,
    1: legTurn,
    2: legForwards,
    3: legTurn,
    4: legForwards,
    5: legTurn,
    6: legForwards,
    7: legTurn,
    8: legForwards,
    9: finished,
    10: stop
    }
# distances
minDistance = 25.0
edgeLimit = 4
# everything based on cycle time
UPDATES = 100                           # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
UPDATE_RATE_MSECS = 1000 * UPDATE_RATE
iterationCount = 0
[leftDistance, frontDistance, rightDistance] = getNewRange()
timeSlots = [0 for i in range(11)]

#delay to get set up
led.on()
while not user_sw.raw():
    pass
led.off()
time.sleep(1)
print(gc.mem_alloc() )
print(gc.mem_free() )
gc.enable
letsGo = True
# Read the encoders until the user button is pressed
while not user_sw.raw() and letsGo:
    timeStart = time.ticks_ms() # get time to calculate loop time
    iterationCount+=1
    [leftDistance, frontDistance, rightDistance] = getNewRange()
    #print(leftDistance, frontDistance, rightDistance)
    #readEncoders()
    if frontDistance > 0:
        
        if legMode <= len(caseStatement):
            caseStatement[legMode]()
        else:
            stop()
    
        
    # calculate cycle delay
    iterTime = time.ticks_diff(time.ticks_ms() , timeStart )
    # determine processing time as multiples of 1 mSec from 1 to 10
    intTime = int(iterTime)
    if intTime > 10:
        intTime = 10
    timeSlots[intTime]+=1
    delayTime = int(UPDATE_RATE_MSECS - iterTime)
    if delayTime > 0 :
        time.sleep_ms(delayTime)
    
print(timeSlots)
print(gc.mem_alloc() )
print(gc.mem_free() )
