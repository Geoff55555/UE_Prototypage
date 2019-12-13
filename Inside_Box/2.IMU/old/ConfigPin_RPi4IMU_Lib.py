import RPi.GPIO as GPIO
import time

def waitForPinConnect(input):
    print("Waiting for Pin BCM nbr "+str(input) +" connection")
    for i in range(65535):
        if GPIO.input(input) == GPIO.LOW:
            print("Pin BCM nbr "+ str(input)+ " connected!")
            return True
        time.sleep(.001) #waits 1ms
    else:
        print("SPI " + str(input)  +" Timeout")
        return False


def beginSPI(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_bcm_spiPortSpeed, user_bcm_spiPort):
    # Get user settings
    print("Initialization")
    _spiPort = user_bcm_spiPort
    _spiPortSpeed = user_bcm_spiPortSpeed

    if _spiPortSpeed > 3000000:
        _spiPortSpeed = 3000000 #BNO080 max SPI freq is 3MHz

    _cs = user_bcm_CSPin
    _wake = user_bcm_WAKPin
    _int = user_bcm_INTPin
    _rst = user_bcm_RSTPin

    print("Setting RPi Pins")
    # Setting RPi pins
    GPIO.setmode(GPIO.BCM) # use BCM numbering (GPIOX)
    GPIO.setup(_cs, GPIO.OUT)
    GPIO.setup(_wake, GPIO.OUT)
    GPIO.setup(_int, GPIO.IN, pull_up_down=GPIO.PUD_UP) # if nothing connected to the input, it will read GPIO.HIGH (pull-up resistor)
    GPIO.setup(_rst, GPIO.OUT)

    # Deselect BNO080
    GPIO.output(_cs, GPIO.HIGH)

    # Config BNO080 for SPI communication
    GPIO.output(_wake, GPIO.HIGH) # Before boot up the pS0/WAK pin mus must be high to enter SPI Mode
    GPIO.output(_rst, GPIO.LOW) # Reset BO080
    time.sleep(0.015) # Waits 15ms for the BNO080 to reset -min length not specified in BNO080 Datasheet-
    GPIO.output(_rst, GPIO.HIGH) # Bring out of reset

    # Now BNO080 will start communicate in SPI
    waitForPinConnect(_int) #wait for connection of INT Pin)


# MAIN
beginSPI(22, 27, 4, 18,1953000 ,0)

print("GPIO cleanup")
GPIO.cleanup()

print("Exiting Program")
