#mazeBurgerBot.py
# motor and encoder libraries
from encoder_rp2 import Encoder as enc # switched to this library rather than Pimoroni
from motor import Motor, pico_motor_shim
from pimoroni import PID, Button
#standard libraries
from machine import Pin
import time
import gc
#range sensors
from hcsr04 import HCSR04
import _thread

# Free up hardware resources ahead of creating a new Encoder
gc.collect()
#create motors
SPEED_SCALE = 1.0
mLeft = Motor(pico_motor_shim.MOTOR_2, direction=0, speed_scale=SPEED_SCALE)
leftEnc = enc(0, Pin(14))
mRight = Motor(pico_motor_shim.MOTOR_1, direction=0, speed_scale=SPEED_SCALE)
rightEnc = enc(1, Pin(16))
# i have created this to aid debugging
# it does show that    i do not need global refernce to leftEnc
def readEncoder(position):
    print("line : {0} value {1}".format(position,leftEnc.value() ))
    mLeft.speed(-0.5)
    time.sleep(1.)
    print(leftEnc.value())
    mLeft.speed(0)


# range sensors
# sonar
# a timeout of 10000 micro secs is 10msecs or 100Hz
# so worst case could take 30 msecs to read all sensors

frontSonar = HCSR04(trigger_pin=18, echo_pin=20, echo_timeout_us=10000)
leftSensor = HCSR04(trigger_pin=10, echo_pin=11, echo_timeout_us=10000)
rightSensor = HCSR04(trigger_pin=13, echo_pin=12, echo_timeout_us=10000)


sonarValues = [-10, -10, -10] # initialise to invalid values [0] is left [1] is forward [2] is right
sonars = [leftSensor, frontSonar , rightSensor]
distanceLock = _thread.allocate_lock()
time.sleep(2) # try a delay to make sure sonars has been defined



# define function for second thread to read all 3 sensors
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

# now start thread
_thread.start_new_thread(readAllSonars, () )

# everything based on cycle time
UPDATES = 100                           # How many times to update the motor per second
UPDATE_RATE = 1 / UPDATES
UPDATE_RATE_MSECS = 1000 * UPDATE_RATE
iterationMax = int(10 / UPDATE_RATE) # 10 seconds
twoSeconds = int(2 / UPDATE_RATE)
modFactor = int(iterationMax/5)
print(modFactor)
# Create PID object for position control
# PID values
POS_KP =0.027# 0.025                           # POS proportional (P) gain
POS_KI = 0.5625#.02                            # POS integral (I) gain
POS_KD = 0.0003375#.0001    
pos_pid_left = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE)
pos_pid_right = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE)
demand = 700#1400#2800
pos_pid_left.setpoint = 0#demand
pos_pid_right.setpoint = 0#demand

iterationCount = 0
maxSpeed = 1
#error = demand - leftEnc.value()
errorThreshold = -1
pastErrors = [ 0 for i in range(300) ]
times = [ 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0]

'''
wheel speed of 1 rev/second
is encoder count of 2800 / second
so position update is 2800 * UPDATE_RATE
top speed about 1.3 revs/sec at wheel
'''
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
offsetrevspersec = 0.014  #speed dependant - offset to move in straight linr
leftrevspersec = revspersec + offsetrevspersec   
rightrevspersec = revspersec
#below required for PID conrol only
defaultSpeed = 2800 * revspersec * UPDATE_RATE
turnSpeed = 2800 * turnrevspersec * UPDATE_RATE
leftSpeed = defaultSpeed
rightSpeed = defaultSpeed
print (" defaultSpeed = {0} turnSpeed = {1}".format(defaultSpeed, turnSpeed) )
stopMode = 0
forwardsMode = 1
moveLeftMode = 2
moveRightMode = 3
mode = forwardsMode
lastmode = -1
# create led
led = Pin(25, Pin.OUT)
# Create the user button
user_sw = Button(pico_motor_shim.BUTTON_A)

# mode definitions and safe distances
minDistance = 25.0
stopMode = 0
forwardsMode = 1
moveLeftMode = 2
modeRightMode = 3
mode = forwardsMode
legModeText = ["Forwards", "Right", "Forwards", "Right", "Forwards", "Left", "Forwards", "Left", "Forwards", "Stop"]
legMode = 0 # Forwards
oldlegMode = -1
startLeftRightTurn = 0
turnDelay = 2
modeText = ["stop","forwards","left","right"]
lastMode = -1


readEncoder(148)

def legForwards():
        global mode, legMode
        # mode logic
        distanceLock.acquire()
        newValues = sonarValues
        distanceLock.release()
    
        frontDistance = newValues[1] 
        leftReading = newValues[0]
        rightReading = newValues[2]
        
        if minDistance < frontDistance < 250  : # safe to move forward
            #print("safe to move forward {0}".format(frontDistance) )
            if mode == moveLeftMode or mode == modeRightMode :
                
                if iterationCount - startLeftRightTurn > turnDelay:
                    mode = forwardsMode
                    leftVel  = leftrevspersec
                    rightVel  = rightrevspersec
                    
            elif edgeLimit > leftReading : # in side left limit so move right
                mode = moveRightMode
                
                startLeftRightTurn = iterationCount
                turnDelay = 0
                leftVel  = leftrevspersec
                rightVel = turnrevspersec
                
            elif rightReading < edgeLimit:
                mode = moveLeftMode
                leftVel  = turnrevspersec
                rightVel = rightrevspersec
                startLeftRightTurn = iterationCount
                turnDelay = 1
            elif mode == forwardsMode : # reset to forwards mode to aid testing
                leftVel  = leftrevspersec
                rightVel = rightrevspersec
            else : # mode must be stop
                mode = forwardsMode
                leftVel= leftrevspersec
                rightVel = rightrevspersec
        else :
            legMode+=1 # need general logic to go onto next leg, turnright and left can be achieved by changing setpoint
            print("frontDistance = {0}".format(frontDistance))
            mode = stopMode
            leftVel = 0
            rightVel = 0
            print("leftEnc {0}".format(leftEnc.value()))
            leftEnc.value(0)
            print("leftEnc {0}".format(leftEnc.value()))
            rightEnc.value(0)
            pos_pid_left.setpoint = 700
            pos_pid_right.setpoint = -700
            
            
        mLeft.speed(-leftVel)
        mRight.speed(-rightVel)


readEncoder(209)
def legTurn(lPos, rPos): # direction specified by setting setpoint
    leftVel = pos_pid_left.calculate(lPos)
    mLeft.speed(leftVel)
    
    rightVel = pos_pid_right.calculate(rPos)
    mRight.speed(rightVel)
    print(lPos, leftVel, rPos, rightVel)
    # add logic to detect when turn complete monitor error ?
    error = (lPos - pos_pid_left.setpoint)**2 + (rPos - pos_pid_right.setpoint)**2
    #print ("error = {0}".format(error) )
    if error < 8: # look for errors < 2 counts in both axes
        print ("error = {0}".format(error) )
        legMode+=1
        leftVel = 0
        rightVel = 0
        leftEnc.value(0)
        rightEnc.value(0)
        pos_pid_left.setpoint = 0
        pos_pid_right.setpoint = 0

readEncoder(230)

#wait for a button press before starting
led.on()
print("press button when ready")
while not user_sw.raw():
    pass
print("pressed")
led.off() # put led off after button has been read
time.sleep(10) # delay to put robot down in course

# get initial values and calculate width, threshaolds etc
distanceLock.acquire()
newValues = sonarValues
distanceLock.release()



edgeLimit = 4

time.sleep(1)

letsGo = True

########################################
tBegin = time.ticks_ms()
#try:
while  (not user_sw.raw()) and letsGo == True :
    timeStart = time.ticks_ms() # get time to calculate loop time
    iterationCount+=1

    if legMode == 0:
        
        legForwards()
        
    elif legMode == 1:
        lPos = leftEnc.value()
        rPos = rightEnc.value()
        print("legmode == 1:{0} {1}".format(lPos, rPos))
        legTurn(lPos, rPos) # try passing encoders through call

    if legMode != oldlegMode:
        print("new legMode= {0}".format(legModeText[legMode]))
        oldlegMode = legMode
    

    if lastmode != mode:
        
        print("mode changed from {0} to {1}".format( modeText[lastmode], modeText[mode] ) )
        lastmode = mode
    
    # calcuate how long this lolop took
    iterTime = time.ticks_diff(time.ticks_ms() , timeStart )
    
    index = int(iterTime)
    
    # capture execution times
    if index > 10:
        index = 10
    times[index] +=1
    
    delayTime = int(UPDATE_RATE_MSECS - iterTime)
    #print(delayTime, UPDATE_RATE_MSECS, iterTime)
    if delayTime > 0 :
        time.sleep_ms(delayTime)
    



#except Exception as ex :
#    print("exception mode = {0} ".format(mode) )
#    template = "An exception of type {0} occurred. Arguments:\n{1!r}"
#    message = template.format(type(ex).__name__, ex.args)
#    print (message)

# while loop has ended so stop motors
mLeft.speed(0)
mRight.speed(0)
# indicate why we stopped
if letsGo:
    led.on()
else:
    led.off()
print(times)
