# Author : Geoffrey ISHIMARU
# References : [1] BNO080 Datasheet from Hillcrest labs, [2] Sparkfun_BNO080.cpp updated by Guillaume Villee, [3] Sparkfun_BNO080.h

import RPi.GPIO as GPIO
import time
import spidev

# Variables to store
# - PINS
_cs = None
_wake = None
_int = None
_rst = None

# - Connexion
spi = None

# - Debug
debugPrint = True

# - Values
SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9  # [3]L59
SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8  # [3]L58

channel = ["Command", "Executable", "Control", "Sensor-report", "Wake-report", "Gyro-vector"]  # Meaning got from [2] L1083

shtpData = []

seqNbr = [0, 0, 0, 0, 0, 0]  # There are 6 comm channels. Each channel has its own seqNum [3]L198


def beginSPI(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_bcm_spiPortSpeed, user_bcm_spiPort):  # strongly inspired from [2]
    # We want the global variable pins declared above
    global _cs, _wake, _int, _rst, spi, shtpData

    # Get user settings
    if debugPrint: print("Initialization")
    _spiPort = user_bcm_spiPort # set to 0 if use of 10 and 11 -MOSI and SCLK- GPIO (SPI0), set to 1 if 20 and 21 (SPI1)
    _spiPortSpeed = user_bcm_spiPortSpeed  # up to 3MHz allowed by BNO but RPi offers 3.9MHz or 1.953MHz (because of clock divider values)

    if _spiPortSpeed > 3000000:
        _spiPortSpeed = 3000000  # BNO080 max SPI freq is 3MHz

    _cs = user_bcm_CSPin
    _wake = user_bcm_WAKPin
    _int = user_bcm_INTPin
    _rst = user_bcm_RSTPin

    if debugPrint: print("Setting RPi Pins")
    # Setting RPi pins
    GPIO.setmode(GPIO.BCM)  # use BCM numbering (GPIOX)
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

    if debugPrint: print('.')  # lighten log

    # Now BNO080 will start communicate in SPI
    if waitForPinResponse(_int): # wait for connection of INT Pin
        # spi launching
        print("[BNO080] Initializing SPI communication")
        spi = spidev.SpiDev() # create spi object
        if debugPrint: print("Opening SPI communication")
        spi.open(_spiPort, 1) # first param = 0 is the bus (corresponding to the spiPort), second param = 1 (?) is the device
        if debugPrint: print("Setting SPI communication parameters")
        spi.mode = 0b11 # https://pypi.org/project/spidev
        print("[BNO080] SPI mode set to : " + str(spi.mode))
        spi.max_speed_hz = _spiPortSpeed
        print("[BNO080] SPI Freq [Hz] set to : " + str(spi.max_speed_hz))

        # Comment from sparkfun : " At system startup, the hub must send its full advertisement message (see 5.2 and 5.3)
        # to the host. It must not send any other data until this step is complete.
        # When BNO080 first boots it broadcasts big startup packet. Read it and dump it."
        waitForPinResponse(_int) # Wait for assertion of INT before reading Init response

        if debugPrint: print("[BNO080] First transmission -not relevant-")
        receiveSPIPacket()
        # print(shtpData)  # just to check if the global shtpData has been used

        # [2] L83 "BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)"
        waitForPinResponse(_int)  # [2] wait for assertion of INT before reading Init response
        if debugPrint: print("[BNO080] Unsolicited Init -not relevant-")
        receiveSPIPacket()
        if debugPrint: print(shtpData)  # just to check if the global shtpData has been used

        # Check communication with the device [2]L100
        shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST  # Request the product ID and reset info
        shtpData[1] = 0                               # Reserved
        if debugPrint: print("check bytes 0 = 0xF9 and byte 1 = 0 :", shtpData)  # just to check if the global shtpData has been used

        # Transmit packet on channel 2 (= control), 2 bytes [2]L103
        if debugPrint: print("SENDING SPI PACKET")
        sendSPIPacket(channel.index("Control"), 2)

        # Now wait for BNO080 response
        waitForPinResponse(_int)
        # Receiving the response
        if debugPrint: print("[BNO080] Response")
        if receiveSPIPacket():
            if debugPrint: print("Data received : " + str(shtpData))
            if shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE:
                if debugPrint: print("[BNO080] Sent back the CORRECT ID response")
                print("[BNO080] Successfully connected via SPI")
                return True  # Will stop here the function

        print("Something went wrong")
        return False


def waitForPinResponse(input):
    if debugPrint: print(".") # lighten log
    if input == 4:
        if debugPrint: print("Waiting INT Pin to trigger")
    else:
        if debugPrint: print("Waiting for Pin BCM nbr "+str(input) +" connection")
    for i in range(2):
        time.sleep(.1) # waits 100ms (under that 100ms it won't connect)
        if GPIO.input(input) == GPIO.LOW:
            if debugPrint:
                if input == 4:
                    print("BNO080 ready!")
                    print(".") # lighten log
                else:
                    print("Pin BCM nbr "+ str(input)+ " connected!")
            return True
        else:
            print("SPI Pin nbr " + str(input) + " couldn't connect. Timeout")
            return False


def receiveSPIPacket():
    global shtpData  # because we want to use the global shtpData defined in the first lines of this code
    if GPIO.input(_int) == GPIO.HIGH:
        print("Data not available")
        return False
    # Select BNO080
    GPIO.output(_cs, GPIO.LOW)
    # Get the first 4 bytes, aka the packet header (in shtp = sensor hub transport protocol from Hillcrest)
    # In SHTP, the first byte is LSB then MSB (while "SPI transmits data MSB first" [1] at the very end of pt 1.3.4.1, if I understood well it is concerning MSB INSIDE the byte, isn't it conventionnaly mandatory?)
    # followed by channel nbr and finally seq nbr [1] pt 1.4.1
    # in [2] L67 it is said "BNO080 has max CLK of 3MHz, MSB first," --> ? So it was source of confusion and little mistake in this code, now corrected
    shtpHeader = spi.readbytes(4)
    packetLSB = shtpHeader[0]
    packetMSB = shtpHeader[1]
    channelNbr = shtpHeader[2]
    seqNbr = shtpHeader[3]
    if debugPrint:
        print("packetLSB = " + str(packetLSB))
        print("packetMSB = " + str(packetMSB))
        try:
            print("Channel nbr = " + str(channelNbr) + " meaning : " + channel[channelNbr])
        except IndexError:
            print("channel number is not recognized")
        print("Sequence nbr = " + str(seqNbr))
    # Calculate the number of data bytes in this packet
    if packetMSB >= 128:
        packetMSB -= 128 # the first bit indicates if this package is continuation of the last. Ignore it for now.
    dataLength = packetMSB*256 + packetLSB # in C++ : ((uint16_t)packetMSB << 8 | packetLSB) --> shift MSB to 8 first bits of (new casted) 16 bits then add 8 LSB.
    if debugPrint: print("Data Length is " + str(dataLength))
    if dataLength == 0:
        return False  # packet empty, done
    # Remove the header bytes from data count
    dataLength -= 4
    # Read incoming data
    shtpData = spi.readbytes(dataLength)
    if debugPrint: print(shtpData)
    if debugPrint: print("Data successfully stored in shtpData")
    # Finishing receive data from BNO080 - Deselect BNO080
    GPIO.output(_cs, GPIO.HIGH)
    # Done !
    return True


def sendSPIPacket(channelNbr, dataLength):  # [2]L1017
    # We want to use global variables
    global _cs, seqNbr
    # packet length contains header (4 bytes) + data
    packetLength = dataLength + 4
    # Wait for BNO080 to indicate (through INTerrupt pin) if it is ready for communication
    if waitForPinResponse(_int) == False:
        if debugPrint: print("BNO080 not ready to receive data")
        return False

    # Select BNO080
    GPIO.output(_cs, GPIO.LOW)

    # 1. Prepare the 4 bytes packet header
    if debugPrint: print("Preparing header to send")
    headerBuffer = [0,0,0,0]
    if packetLength < 256:
        headerBuffer[0] = packetLength  # packet length LSB
        headerBuffer[1] = 0             # packet length MSB
    elif packetLength >= 256:
        headerBuffer[0] = packetLength % 256  # packet length LSB
        headerBuffer[1] = packetLength // 256  # packet length MSB

    headerBuffer[2] = channelNbr
    headerBuffer[3] = seqNbr[channelNbr]
    seqNbr[channelNbr] =  seqNbr[channelNbr] + 1

    # 2. Send the header to BNO080
    if debugPrint: print("Sending headerBuffer : " + str(headerBuffer))
    spi.writebytes(headerBuffer)

    # 3. Prepare user's data packet
    buffer = shtpData
    # 4. Send user's data to BNO080
    if debugPrint: print("Sending data : " + str(buffer))
    spi.writebytes(buffer)

    # Deselect BNO080
    GPIO.output(_cs, GPIO.HIGH)

    if debugPrint: print('.')  # lighten log
    # We're done!
    return True


# MAIN
try:
    beginSPI(22, 27, 4, 18, 1953000, 0)  # freq for Linux Kernel 1953000, but BNO080 can go up to 3000000
    waitForPinResponse(_int)
    receiveSPIPacket()
    print("Everything went SMOOTH")
except Exception as e:
    print("An error occurred. Error message : " + str(e))
finally:
    print('.')  # lighten log
    print("Closing SPI communication")
    spi.close()
    print("GPIO cleanup")
    GPIO.cleanup()

    print("Exiting Program")
