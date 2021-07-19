#### PART 1: (IMPORTING LIBRARIES)
import RPi.GPIO as GPIO                     # Library for accessing RASPBERRY PI GPIO’S
import time                                 # Library for importing delay (i.e. import delay)
import serial                               # Library for enabling serial communication
import pigpio                               # Library for Hardware servo Library
import sys                                  # Library for Keyboard Press function
import termios                              # Library for Keyboard Press function
import tty                                  # Library for Keyboard Press function
import os                                   # Library for Running Linux terminal commands
import pygame                               # Library used for Playing Voice
import requests                             # Library for HTTP Requests

#### PART 2: (INITIALIZATION & PINS DECLARATION)
pi = pigpio.pi()                            # Way of initializing Hardware servo library
pygame.init()                               # Way of initializing pygame libary

#HEAD SERVO# > Comment starting with #
head = 24 > Variable declaration (no rocket science here)
h = 1240 > same

#MOVEMENT MOTORS#
#motor1
m1pwm = 10
m1a = 9
m1b = 25
#motor2
#m2pwm = 27
#m2a = 22
#m2b = 23
#motor2.1
#motor1
m2pwm = 4
m2a = 18
m2b = 17

#ARM MOTORS#
#LDutyCycle motor
m4pwm = 13
m4a = 19
m4b = 16
#RDutyCycle motor
m5pwm = 21
m5a = 20
m5b = 26

#LDutyCycle servo
mlpwm = 12
l = 1700 # <<---CHANGE THIS---------
#RDutyCycle servo
mrpwm = 6
r = 500

#initial setting
GPIO.setmode(GPIO.BCM)                      # Mode SET for using BCM GPIO PINS
GPIO.setwarnings(False)                     # GPIO Warnings SET to False
#PINS DECLARATIONS#
GPIO.setup(m1a,GPIO.OUT)                    # GPIO MOTOR1 FOR MOVEMENT
GPIO.setup(m1b,GPIO.OUT)
GPIO.setup(m2a,GPIO.OUT)                    # GPIO MOTOR2 FOR MOVEMENT
GPIO.setup(m2b,GPIO.OUT)
GPIO.setup(m4a,GPIO.OUT)                    # GPIO MOTOR4 FOR ARM
GPIO.setup(m4b,GPIO.OUT)
GPIO.setup(m5a,GPIO.OUT)                    # GPIO MOTOR5 FOR ARM
GPIO.setup(m5b,GPIO.OUT)

#PWM SETTINGS#
GPIO.setup(m1pwm,GPIO.OUT)                  # GPIO MOTOR1 PWM
GPIO.setup(m2pwm,GPIO.OUT)                  # GPIO MOTOR2 PWM
GPIO.setup(m4pwm,GPIO.OUT)                  # GPIO MOTOR4 PWM
GPIO.setup(m5pwm,GPIO.OUT)                  # GPIO MOTOR5 PWM
m4p = GPIO.PWM(m4pwm,50)                    # FREQUENCY 50Hz FOR ARM MOTOR
m5p = GPIO.PWM(m5pwm,50)                    # FREQUENCY 50Hz FOR ARM MOTOR

#servo motor pwm
GPIO.setup(mlpwm,GPIO.OUT)                  # SERVO MOTOR LEFT
pi.set_servo_pulsewidth(12,l)               # PIN12 set to l = 1700 DUTY CYCLE
pass                                        # FOR STABILITY OF SERVO
pass
GPIO.setup(mrpwm,GPIO.OUT)                  # SERVO MOTOR RIGHT
pi.set_servo_pulsewidth(6,r)                # PIN6 set to r = 500 DUTY CYCLE
pass
pass
GPIO.setup(head,GPIO.OUT)                   # SERVO MOTOR HEAD
pi.set_servo_pulsewidth(7,h)                # PIN7 set to h = 1240 DUTY CYCLE
pass
pass
pass

#pwm initial cycle
pwm_1p = 100 > VARIABLE
pwm_2p = 80
#global x
x=0
PwmLocomotion = 245
Condition = 0

#### PART 3: (PWM DECLARATION)
#pi.set_PWM_frequency(m1pwm,5000) > FREQUENCY 5000Hz (COMMENTED)
pi.set_PWM_dutycycle(m1pwm,240) # DUTY CYCLE 240
#pi.set_PWM_frequency(m2pwm,5000) > FREQUENCY 5000Hz (COMMENTED)
pi.set_PWM_dutycycle(m2pwm,255) # DUTY CYCLE 255
m4p.start(100) # MOTOR4 INITIAL PWM 100
m5p.start(100) # MOTOR5 INITIAL PWM 100

#### PART 4: (SERIAL COMMUNICATION SETTINGS)
#Serial communication settings# > PARAMETERS FOR SERIAL
ser = serial.Serial('/dev/ttyS0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS) # /dev/ttyS0 COM PORT in LINUX

#### PART 5: (METHOD FOR CATCHING KEYBOARD KEY PRESS)
#Function for catching key#

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
    finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0

## note: getch() = VARIABLE. WHICH WILL CONTAIN VALUE OF KEY THAT IS PRESSED

#### PART 6: (PROGRAM STARTING)
comm = None
print("Program start")
try:                                    # TRY BLOCK FOR EXCEPTION ERROR
    while(True):                        # INFINITE BLOCK

    #comm = requests.get('https://api.thingspeak.com/apps/thinghttp/send_request?api_key=CARKYI86ODDHIOW4') > UNCOMMENT FOR THINGPEAK CONTROL
    #comm = getch() > UNCOMMENT FOR KEYBOARD KEY PRESS
    #comm = raw_input() > UNCOMMENT FOR KEY PRESS + ENTER
        while(ser.inWaiting() > 0): # UNCOMMENT FOR SERIAL CONTROL
            comm = ser.read()

#### PART 7: (IF ELSE STATEMENTS FOR LOCOMOTION)
            if comm == 'F':
                print "Forward"
                pi.set_PWM_dutycycle(m1pwm,240)
                pi.set_PWM_dutycycle(m2pwm,255)
                GPIO.output(m1a,0) #stop
                GPIO.output(m1b,1)
                GPIO.output(m2a,0) #forward
                GPIO.output(m2b,1)
                #Condition = 1
                ## while(k < 255):
                ## pi.set_PWM_dutycycle(m1pwm,k)
                ## pi.set_PWM_dutycycle(m2pwm,k)
                ## k=k+20
                ## time.sleep(0.005)
                ## print k
                #speed.up()
            elif comm == 'B':
                print "Backward"
                pi.set_PWM_dutycycle(m1pwm,100)
                pi.set_PWM_dutycycle(m2pwm,100)
                GPIO.output(m1a,1) #LDutyCycle
                GPIO.output(m1b,0)
                GPIO.output(m2a,1) #stop
                GPIO.output(m2b,0)
                #Condition = 1
            elif comm == 'R': #RIGHT
                print "Right"
                pi.set_PWM_dutycycle(m1pwm,255)
                pi.set_PWM_dutycycle(m2pwm,255)
                GPIO.output(m1a,0) #LDutyCycle
                GPIO.output(m1b,1)
                GPIO.output(m2a,1) #stop
                GPIO.output(m2b,0)
                #Condition = 1
            elif comm == 'L': #LEFT
                print "Left"
                pi.set_PWM_dutycycle(m1pwm,255)
                pi.set_PWM_dutycycle(m2pwm,255)
                GPIO.output(m1a,1) #RDutyCycle
                GPIO.output(m1b,0)
                GPIO.output(m2a,0) #backward
                GPIO.output(m2b,1)
                #Condition = 1
                ## elif comm == 'h':
                ## print "Clockwise"
                ## GPIO.output(m1a,0) #LDutyCycle
                ## GPIO.output(m1b,1)
                ## GPIO.output(m2a,0) #backward
                ## GPIO.output(m2b,0)
                ## elif comm == 'h':
                ## print "Anticlockwise"
                ## GPIO.output(m1a,0) #RDutyCycle
                ## GPIO.output(m1b,0)
                ## GPIO.output(m2a,0) #forward
                ## GPIO.output(m2b,1)
            
#### PART 8: (IF ELSE STATEMENTS FOR SERVOS)
            elif comm == '#':               # IF # RECEIVED
                prev = h                    # PREV VALUE = h (SAVE ORIGINAL VALUE)
                h = h + 20                  # NEW VALUE = + 20 (CHANGINE VALUE)
                print(prev,h)               # PRINT PREV + h
                time.sleep(0.05)            # DELAY
                if h >=500 and h <= 1500:   # RANGE FOR LIMITING SERVO
                    pi.set_servo_pulsewidth(7,h) # NEW VALUE EXECUTE
                    print("H")                   # PRINT H
                else:                       # ELSE IF VALUE NOT IN RANGE
                    h = prev                # H = PREV VALUE (RETURN ORG.)
                    print("N H")            # PRINT N H
            elif comm == 'b':
                prev = h
                h = h - 20
                print(prev,h)
                time.sleep(0.05)
                if h >=500 and h <= 1500:
                    pi.set_servo_pulsewidth(7,h)
                    print("H")
                else:
                    h = prev
                    print("N H")
            elif comm == 'c':
                prev = r
                r = r + 20
                print(prev,r)
                time.sleep(0.025)
                if r >= 500 and r <= 1700:
                    # pi.set_PWM_dutycycle(6,r)
                    pi.set_servo_pulsewidth(6,r)
                    # time.sleep(0.35)
                    print("D")
                else:
                    r = prev
                    print("N")
            elif comm == 'd':
                prev = r
                r = r - 20
                time.sleep(0.025)
                print(prev,r)
                if r >= 500 and r <= 1700:
                    #pi.set_PWM_dutycycle(6,r)
                    pi.set_servo_pulsewidth(6,r)
                    print("D")
                else:
                    r = prev
                    print("N")
            elif comm == 'f':
                prev = l
                l = l + 20
                print(prev,l)
                time.sleep(0.025)
                if l >= 500 and l <= 1700:
                    # pi.set_PWM_dutycycle(6,r)
                    pi.set_servo_pulsewidth(12,l)
                    # time.sleep(0.35)
                    print("F")
                else:
                    l = prev
                    print("M")
            elif comm == 'e':
                prev = l
                l = l - 20
                print(prev,l)
                time.sleep(0.025)
                if l >= 500 and l <= 1700:
                    #pi.set_PWM_dutycycle(6,r)
                    pi.set_servo_pulsewidth(12,l)
                    print("F")
                else:
                    l = prev
                    print("M")

#### PART 9: (IF ELSE STATEMENTS FOR DC ARM MOTORS)
            elif comm == 't':
                print("Left arm motor increment")
                GPIO.output(m4a,0)
                GPIO.output(m4b,1)
                time.sleep(0.5)
                #GPIO.output(m4b,0)
            elif comm == 'y':
                print("Left arm motor decrement")
                GPIO.output(m4a,1)
                GPIO.output(m4b,0)
                time.sleep(0.5)
                #GPIO.output(m4a,0)
            elif comm == 'm':
                print("Right arm motor increment")
                GPIO.output(m5a,0)
                GPIO.output(m5b,1)
                time.sleep(0.5)
                #GPIO.output(m5b,0)
            elif comm == 'n':
                print("Right arm motor decrement")
                GPIO.output(m5a,1)
                GPIO.output(m5b,0)
                time.sleep(0.5)
                #GPIO.output(m5a,0)

#### PART 10: (IF ELSE STATEMENTS FOR VOICE)
            elif comm == 'h':                   # IF h RECEIVED
                print("sound PROCOM")           # PRINT
                pygame.mixer.music.load("procom.mp3") # LOAD SOUND FILE
                pygame.mixer.music.play()       # PLAY IT
            elif comm == 'j':
                print("sound PROCOM 2")
                pygame.mixer.music.load("pro_2.mp3") #other
                pygame.mixer.music.play()
            elif comm == 'k':
                print("sound PROCOM 3")
                pygame.mixer.music.load("pro_3.mp3") #other
                pygame.mixer.music.play()
                ## time.sleep(2)
                ## for r in range(500,1700,1):
                ## pi.set_servo_pulsewidth(6,r)
                ## time.sleep(0.0025)
                ## time.sleep(1)
                ## for r in range(500,1700,1):
                ## pi.set_servo_pulsewidth(6,2200-r)
                ## time.sleep(0.003)
            elif comm == 'a':
                pygame.mixer.music.load("else.mp3")
                pygame.mixer.music.play()
            elif comm == 'l':                       # IF l RECEIVED
                pygame.mixer.music.stop()           # STOP, WHATEVER IS PLAYING

#### PART 11: (PROGRAM EXIT)
            elif comm == 'p':                       # p = EXIT FOR KEYBOARD KEY PRESS
                print("Program Exit")
                exit(0)
            elif comm == 's' or comm == 'S':        # FOR ANDROID APP S or s = EXIT

                #speed.down()
                #while(k > 0):
                # pi.set_PWM_dutycycle(m1pwm,k)
                # pi.set_PWM_dutycycle(m2pwm,k)
                # pi.set_PWM_dutycycle(m1pwm,k)
                # k=k-1
                # time.sleep(0.005)
                # print k
                PwmLocomotion = 245
                GPIO.output(m1a,0)
                GPIO.output(m1b,0)
                GPIO.output(m2a,0)
                GPIO.output(m2b,0)
                GPIO.output(m4a,0)
                GPIO.output(m4b,0)
                GPIO.output(m5a,0)
                GPIO.output(m5b,0)
                print("OUTPUT Stopped")
                #GPIO.output(mlpwm,0)
                #GPIO.output(mrpwm,0)

except KeyboardInterrupt:                       # IF CTRL + C, PRESSED
    pass                                        # IGNORE IT

ser.close()                                     # AND CLOSE SERIAL COMMUNICATION
#m2p.stop()
#m1p.stop()
m4p.stop()                                      # STOP PWM
m5p.stop()
GPIO.cleanup()                                  # CLEAN GPIO’S