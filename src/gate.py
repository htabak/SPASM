from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

class gateSubsystem:
    def __init__ (self):
        pass

    def open (self):
        kit.servo[0].angle = 25

    def close (self):
        kit.servo[0].angle = 90

    def moveServo (self, angle):
        kit.servo[0].angle = angle
