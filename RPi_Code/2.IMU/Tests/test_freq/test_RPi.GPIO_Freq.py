import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

GPIO.setup(4, GPIO.OUT)

for i in range(10000000):
    GPIO.output(4, True)
    GPIO.output(4, False)

GPIO.cleanup()
