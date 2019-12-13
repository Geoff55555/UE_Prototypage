import RPi.GPIO as GPIO
import time
import spidev

def waitForPinConnect(input):
    print(".") # lighten log
    if input == 4:
        print("Waiting INT Pin to trigger")
    else:
        print("Waiting for Pin BCM nbr "+str(input) +" connection")
    for i in range(2):
        time.sleep(.1) # waits 100ms (under that 100ms it won't connect)
        if GPIO.input(input) == GPIO.LOW:
            if input==4:
                print("BNO080 ready!")
                print(".") # lighten log
            else:
                print("Pin BCM nbr "+ str(input)+ " connected!")
            return True
        else:
            print("SPI Pin nbr " + str(input)  +" couldn't connect. Timeout")
            return False


def receiveSPIPacket(spi_Obj, _intPin, _csPin):
    if GPIO.input(_intPin) == GPIO.HIGH:
        print("Data not available")
        return False
    # Select BNO080
    GPIO.output(_csPin, GPIO.LOW)
    # Get the first 4 bytes, aka the packet header
    header_info = spi_Obj.readbytes(4)
    packetMSB = header_info[0]
    packetLSB = header_info[1]
    channelNbr = header_info[2]
    seqNbr = header_info[3]
    print("packetLSB = " + str(packetMSB))
    print("packetMSB = " + str(packetLSB))
    print("Channel nbr = " + str(channelNbr))
    print("Sequence nbr = " + str(seqNbr))
    # Calculate the number of data bytes in this packet
    if packetMSB >= 128:
        packetMSB -= 128 # the first bit indicates if this package is continuation of the last. Ignore it for now.
    dataLength = packetMSB*256 + packetLSB # in C++ : ((uint16_t)packetMSB << 8 | packetLSB) --> shift MSB to 8 first bits of (new casted) 16 bits then add 8 LSB.
    print("Data Length is " + str(dataLength))
    if dataLength == 0:
        return False # packet empty, done
    # Remove the header bytes from data count
    dataLength -= 4
    # Read incoming data
    data = spi_Obj.readbytes(dataLength)
    print(data)

def beginSPI(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_bcm_spiPortSpeed, user_bcm_spiPort):
    # Get user settings
    print("Initialization")
    _spiPort = user_bcm_spiPort # set to 0 if use of 10 and 11 -MOSI and SCLK- GPIO (SPI0), set to 1 if 20 and 21 (SPI1)
    _spiPortSpeed = user_bcm_spiPortSpeed # up to 3MHz allowed by BNO but RPi offers 3.9MHz or 1.953MHz (because of clock divider values)

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
    if waitForPinConnect(_int): # wait for connection of INT Pin
        # spi launching
        print("Initializing SPI communication")
        spi = spidev.SpiDev() # create spi object
        print("Opening SPI communication")
        spi.open(0, 1) # fisrt param = 0 is the bus (corresponding to the spiPort), second param = 1 (?) is the device
        print("Setting SPI communication parameters")
        spi.mode = 0b11 # https://pypi.org/project/spidev
        print("SPI mode set to : " + str(spi.mode))
        spi.max_speed_hz = _spiPortSpeed
        print("SPI Freq [Hz] set to : " + str(spi.max_speed_hz))

        # Comment from sparkfun : " At system startup, the hub must send its full advertisement message (see 5.2 and 5.3)
        # to the host. It must not send any other data until this step is complete.
        # When BNO080 first boots it broadcasts big startup packet. Read it and dump it."
        waitForPinConnect(_int) # Wait for assertion of INT before reading Init response

        receiveSPIPacket(spi, _int, _cs)

        print("Everything went SMOOTH")
        print("Closing spi communication")
        spi.close()


# MAIN
try:
    beginSPI(22, 27, 4, 18, 1953000, 0)
except Exception as e:
    print("An error occurred. Error message : " + str(e))
finally:
    print("GPIO cleanup")
    GPIO.cleanup()

    print("Exiting Program")
