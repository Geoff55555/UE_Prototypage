# Author : Geoffrey ISHIMARU
# References : [1] BNO080 Datasheet from Hillcrest labs, [2] Sparkfun_BNO080.cpp updated by Guillaume Villee, [3] Sparkfun_BNO080.h, [4] Reference Manual from Hillcrestlabs

import RPi.GPIO as GPIO
import time
import spidev

# -------------------------------------------------------------------
# ---------------------- VARIABLES TO STORE -------------------------
# -------------------------------------------------------------------

# - PINS
_cs = None
_wake = None
_int = None
_rst = None

# - Connexion
spi = None

# - Debug
debugPrint = False
timeStamp = 0  #32-bit value

# - Low Level Communication with sensor
SHTP_REPORT_COMMAND_RESPONSE = 0xF1
SHTP_REPORT_COMMAND_REQUEST = 0xF2
SHTP_REPORT_PRODUCT_ID_REQUEST = 0xF9  # [3]L59
SHTP_REPORT_PRODUCT_ID_RESPONSE = 0xF8  # [3]L58
SHTP_REPORT_BASE_TIMESTAMP = 0xFB  # [3]L60
SHTP_REPORT_FRS_READ_REQUEST = 0xF4  # [3]L57
SHTP_REPORT_FRS_READ_RESPONSE = 0xF3  # [3]L56
SHTP_REPORT_SET_FEATURE_COMMAND = 0xFD  # [3]L61

SENSOR_REPORTID_ACCELEROMETER = 0x01  # [3] beginning from L65 and for functionparseInputReport at L212
SENSOR_REPORTID_GYROSCOPE = 0x02
SENSOR_REPORTID_MAGNETIC_FIELD = 0x03
SENSOR_REPORTID_LINEAR_ACCELERATION = 0x04
SENSOR_REPORTID_ROTATION_VECTOR = 0x05
SENSOR_REPORTID_GRAVITY = 0x06
SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08
SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR = 0x09
SENSOR_REPORTID_STEP_COUNTER = 0x11
SENSOR_REPORTID_STABILITY_CLASSIFIER = 0x13
SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E

COMMAND_ME_CALIBRATE = 7  # [2] L92

channel = ["Command", "Executable", "Control", "Sensor-report", "Wake-report", "Gyro-vector"]  # Meaning got from [2] L1083

# - Communication Data Storage
shtpHeader = []
shtpData = []
dataLength = 0  # length of shtpData
seqNbr = [0, 0, 0, 0, 0, 0]  # There are 6 comm channels. Each channel has its own seqNum [3]L198

# - Raw sensor values
rawAccelX, rawAccelY, rawAccelZ, accelAccuracy = None,None,None,None  # [3] L214
rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy = None,None,None,None
rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy = None,None,None,None
rawMagX, rawMagY, rawMagZ, magAccuracy = None,None,None,None
rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy = None,None,None,None,None,None
stepCount = None
timeStamp = None
stabilityClassifier = None
activityClassifier = None

_activityConfidence = []  # Array that store the confidences of the 9 possible activities
for i in range(9):
    _activityConfidence.append(None)
    
calibrationStatus = None  # Byte R0 of ME Calibration Response

rotationVector_Q1 = 14  # [3]L227 These Q values are defined in the datasheet but can also be obtained by querying the meta data records
accelerometer_Q1 = 8
linear_accelerometer_Q1 = 8
gyro_Q1 = 9
magnetometer_Q1 = 4


# -------------------------------------------------------------------
# --------------------- COMMUNICATION INIT --------------------------
# -------------------------------------------------------------------

def beginSPI(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_bcm_spiPortSpeed, user_bcm_spiPort):  # strongly inspired from [2]
    # We want the global variable pins declared above
    global _cs, _wake, _int, _rst, spi, shtpData

    # Get user settings
    if debugPrint: print("[BNO080] Setting up SPI communication")
    _spiPort = user_bcm_spiPort # set to 0 if use of 10 and 11 -MOSI and SCLK- GPIO (SPI0), set to 1 if 20 and 21 (SPI1)
    _spiPortSpeed = user_bcm_spiPortSpeed  # up to 3MHz allowed by BNO but RPi offers 3.9MHz or 1.953MHz (because of clock divider values)

    if _spiPortSpeed > 3000000:
        _spiPortSpeed = 3000000  # BNO080 max SPI freq is 3MHz

    _cs = user_bcm_CSPin
    _wake = user_bcm_WAKPin
    _int = user_bcm_INTPin
    _rst = user_bcm_RSTPin

    if debugPrint: print("[BNO080] Setting up RPi Pins")
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
        if debugPrint: print("[BNO080] Opening SPI communication")
        spi.open(_spiPort, 1) # first param = 0 is the bus (corresponding to the spiPort), second param = 1 (?) is the device
        if debugPrint: print("[BNO080] Setting SPI communication parameters")
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

        # Check communication with the device [2]L100 / [4] Product ID Request p.37 Figure 35 and Check the response p.38
        shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST  # Request the product ID and reset info
        shtpData[1] = 0                               # Reserved
        if debugPrint: print("[BNO080] RPi changing shtpData to send test packet")
        if debugPrint: print("[BNO080] Check bytes 0 = 0xF9 and byte 1 = 0 :", shtpData)  # just to check if the global shtpData has been used

        # Transmit packet on channel 2 (= control), 2 bytes [2]L103
        if debugPrint: print("[BNO080] RPi SENDING SPI PACKET")
        sendSPIPacket(channel.index("Control"), 2)

        # Now wait for BNO080 response
        waitForPinResponse(_int)
        # Receiving the response / [4] Check the response p38: bit0 = 0xF8 = SHTP_REPORT_PRODUCT_ID_RESPONSE
        if debugPrint: print("[BNO080] Response")
        if receiveSPIPacket():
            if debugPrint: print("[BNO080] Data received : " + str(shtpData))
            if shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE:
                if debugPrint: print("[BNO080] sent back the CORRECT ID response")
                print("[BNO080] Successfully connected via SPI")
                return True  # Will stop here the function

        print("[BNO080] Something went wrong")
        raise RuntimeError("[BNO080] did NOT send back the CORRECT ID response")
        return False

# -------------------------------------------------------------------
# ------------------- ASK/RECEIVE SPI PACKETS -----------------------
# -------------------------------------------------------------------

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
                    print("[BNO080] ready!")
                    print(".") # lighten log
                else:
                    print("[BNO080] RPi Pin BCM nbr "+ str(input)+ " connected!")
            return True
        else:
            print("[BNO080] RPI Pin nbr " + str(input) + " couldn't connect. Timeout")
            return False


def printHeader():
    print("[BNO080] packetLSB = " + str(shtpHeader[0]))
    print("[BNO080] packetMSB = " + str(shtpHeader[1]))
    try:
        print("[BNO080] Channel nbr = " + str(shtpHeader[2]) + " meaning : " + channel[shtpHeader[2]])
    except IndexError:
        print("[BNO080] Channel number is not recognized")
    print("[BNO080] Sequence nbr = " + str(shtpHeader[3]))


def receiveSPIPacket():
    global shtpData, shtpHeader, dataLength  # because we want to use the global shtpData defined in the first lines of this code
    if GPIO.input(_int) == GPIO.HIGH:
        print("[BNO080] Data not available")
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
    if debugPrint: printHeader()
    # Calculate the number of data bytes in this packet
    if packetMSB >= 128:
        packetMSB -= 128 # the first bit indicates if this package is continuation of the last. Ignore it for now.
    dataLength = packetMSB*256 + packetLSB # in C++ : ((uint16_t)packetMSB << 8 | packetLSB) --> shift MSB to 8 first bits of (new casted) 16 bits then add 8 LSB.
    if debugPrint: print("[BNO080] Data Length (including header) is " + str(dataLength))
    if dataLength == 0:
        return False  # packet empty, done
    # Remove the header bytes from data count --> it is only length of data, not the packet
    dataLength -= 4
    # Read incoming data
    shtpData = spi.readbytes(dataLength)
    if debugPrint: print(shtpData)
    if debugPrint: print("[BNO080] Data successfully stored in shtpData")
    # Finishing receive data from BNO080 - Deselect BNO080
    GPIO.output(_cs, GPIO.HIGH)
    # Done !
    return True


def sendSPIPacket(channelNbr, dataLength):  # [2]L1017
    # We want to use global variables to be able to write and save the changes about them
    global seqNbr
    # packet length contains header (4 bytes) + data
    packetLength = dataLength + 4
    # Wait for BNO080 to indicate (through INTerrupt pin) if it is ready for communication
    if waitForPinResponse(_int) == False:
        if debugPrint: print("[BNO080] not ready to receive data")
        return False

    # Select BNO080
    GPIO.output(_cs, GPIO.LOW)

    # 1. Prepare the 4 bytes packet header
    if debugPrint: print("[BNO080] RPi Preparing header to send")
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
    if debugPrint: print("[BNO080] RPi Sending headerBuffer : " + str(headerBuffer))
    spi.writebytes(headerBuffer)

    # 3. Prepare user's data packet
    buffer = shtpData
    # 4. Send user's data to BNO080
    if debugPrint: print("[BNO080] RPi Sending data : " + str(buffer))
    spi.writebytes(buffer)

    # Deselect BNO080
    GPIO.output(_cs, GPIO.HIGH)

    if debugPrint: print('.')  # lighten log
    # We're done!
    return True

# -------------------------------------------------------------------
# ------------------------ ASK USEFUL DATA --------------------------
# -------------------------------------------------------------------

def dataAvailable():  # [2]L130
    if _int != None:  # Pre checks before launching any function about data
        if receiveSPIPacket() == True:
            # We want to know what kind of packet this is
            try:
                if (shtpHeader[2] == channel.index("Sensor-report") and shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP):
                    print("[BNO080] has data available to send from its sensors !")
                    parseInputReport()
                    return True
                elif shtpHeader[2] == channel.index("Control"):
                    print("[BNO080] sent reponses about the RPi commands (status, etc.) - NO SENSOR DATA")
                    parseCommandReport()
                    return True
            except IndexError:
                print("[BNO080] Channel number is not recognized")
            return False

# Given a sensor's report ID, this tells the BNO080 to begin reporting the values
# Also sets the specific config word. Useful for personal activity classifier
def setFeatureCommandSpecConf(reportID, timeBetweenReports, specificConfig):  # [2] L794 -- Time in milliseconds
    microsBetweenReports = timeBetweenReports * 1000  # (32-bit nbr)
    # Making shtpData array bigger to avoid index out of range
    shtpData.clear()
    for i in range(18):  # 0 to 18 not included
        shtpData.append(0)
    # Preparing packet
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND     # set feature command [4]p.55+56
    shtpData[1] = reportID                            # Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0                                   # Feature flags
    shtpData[3] = 0                                   # Change sensitivity (LSB)
    shtpData[4] = 0                                   # Change sensitivity (MSB)
    shtpData[5] = microsBetweenReports % 256          # Report Interval (LSB from 32-bit) in microseconds. 0x7A120 = 500ms
    shtpData[6] = microsBetweenReports // 256         # Report Interval
    shtpData[7] = microsBetweenReports // pow(256,2)  # Report Interval
    shtpData[8] = microsBetweenReports // pow(256,3)  # Report Interval (MSB)
    shtpData[9] = 0                                   # Batch Interval (LSB)
    shtpData[10] = 0                                  # Batch Interval
    shtpData[11] = 0                                  # Batch Interval
    shtpData[12] = 0                                  # Batch Interval (MSB)
    shtpData[13] = specificConfig % 256               # Sensor-specific config (LSB)
    shtpData[14] = specificConfig // 256              # Sensor-specific config
    shtpData[15] = specificConfig // pow(256,2)       # Sensor-specific config
    shtpData[16] = specificConfig // pow(256,3)       # Sensor-specific config (MSB)
    # Sending packet on channel 2, 17 bytes
    try:
        sendSPIPacket(channel.index("Control"), 17)
    except IndexError:
        print("[BNO080] Channel number is not recognized in setFeatureCommandSpecConf")


# Given a sensor's report ID, this tells the BNO080 to begin reporting the values
def setFeatureCommand(reportID, timeBetweenReports):  # [2] L787
    setFeatureCommandSpecConf(reportID, timeBetweenReports, 0)  # No specific config


# Sends the packet to enable the magnetometer
def enableMagnetometer(timeBetweenReports):  #[2] L717
    setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports)


# Sends the packet to enable the rotation vector --> return a quaternion = magnetometer + accelerometer + gyroscope /!\ enable magnetometer first !
def enableRotationVector(timeBetweenReports):  #[2] L689
    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports)


# -------------------------------------------------------------------
# ------------------ USEFUL DATA INTERPRETATION ---------------------
# -------------------------------------------------------------------

def parseInputReport():  # [2]L197
    # Because we want to access and modify the global variables
    global rawAccelX, rawAccelY, rawAccelZ, accelAccuracy
    global rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy
    global rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy
    global rawMagX, rawMagY, rawMagZ, magAccuracy
    global rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy
    global stepCount, timeStamp
    global stabilityClassifier, activityClassifier, _activityConfidence, calibrationStatus
    
    timeStamp = shtpData[4]*pow(256,3) + shtpData[3]*pow(256,2) + shtpData[2]*256 + shtpData[1]

    status = shtpData[5 + 2] & 0x03
    data1 = shtpData[5 + 5]*256 + shtpData[5 + 4]
    data2 = shtpData[5 + 7]*256 + shtpData[5 + 6]
    data3 = shtpData[5 + 9]*256 + shtpData[5 + 8]
    data4 = 0
    data5 = 0

    if dataLength - 5 > 9:
        data4 = shtpData[5 + 11]*256 + shtpData[5 + 10]
    if dataLength - 5 > 11:
        data5 = shtpData[5 + 13]*256 + shtpData[5 + 12]
    # Store these generic values to their proper global variable
    if shtpData[5] == SENSOR_REPORTID_ACCELEROMETER:
        print("SENSOR_REPORTID_ACCELEROMETER")
        accelAccuracy = status
        rawAccelX = data1
        rawAccelY = data2
        rawAccelZ = data3

    elif shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION:
        print("SENSOR_REPORTID_LINEAR_ACCELERATION")
        accelLinAccuracy = status
        rawLinAccelX = data1
        rawLinAccelY = data2
        rawLinAccelZ = data3

    elif shtpData[5] == SENSOR_REPORTID_GYROSCOPE:
        print("SENSOR_REPORTID_GYROSCOPE")
        gyroAccuracy = status
        rawGyroX = data1
        rawGyroY = data2
        rawGyroZ = data3

    elif shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD:
        print("SENSOR_REPORTID_MAGNETIC_FIELD")
        magAccuracy = status
        rawMagX = data1
        rawMagY = data2
        rawMagZ = data3

    elif shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR or shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR:
        print("SENSOR_REPORTID_ROTATION_VECTOR or SENSOR_REPORTID_GAME_ROTATION_VECTOR")
        quatAccuracy = status
        rawQuatI = data1
        rawQuatJ = data2
        rawQuatK = data3
        rawQuatReal = data4
        rawQuatRadianAccuracy = data5  # only available on rotation vector, not game rot vect

    elif shtpData[5] == SENSOR_REPORTID_STEP_COUNTER:
        print("SENSOR_REPORTID_STEP_COUNTER")
        stepCount = data3  # Bytes 8/9

    elif shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER:
        print("SENSOR_REPORTID_STABILITY_CLASSIFIER")
        stabilityClassifier = shtpData[5 + 4]  # Byte 4 only

    elif shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
        print("SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER")
        activityClassifier = shtpData[5 + 5]  # Most likely state
        # Load activity classification confidences into the array
        for x in range(9):
            _activityConfidence[x] = shtpData[5 + 6 + x]

    elif shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE:
        print("SHTP_REPORT_COMMAND_RESPONSE")
        # The BNO080 responds with this report to command requests. It's up to use to remember which command we used
        command = shtpData[5 + 2]  # This is the command byte of the response
        if command == COMMAND_ME_CALIBRATE:
            print("[BNO080] has sent a calibration report")
            calibrationStatus = shtpData[5 + 5]  # R0 - Status (0 = success, non-zero = fail)

    else:
        print("Sensor Report ID not handled")

    return True


def parseCommandReport():
    return True

# -------------------------------------------------------------------
# ---------------------- GET EVERYTHING -----------------------------
# -------------------------------------------------------------------

# Given a register value and a Q point, convert to float [2] L677
def qToFloat(fixedPointValue, qPoint):
    qFloat = fixedPointValue
    qFloat *= pow(2, qPoint * - 1)
    return qFloat

# Return the rotation vector quaternion I
def getQuatI():
    quat = qToFloat(rawQuatI, rotationVector_Q1)
    return quat

    
def getQuatJ():
    quat = qToFloat(rawQuatJ, rotationVector_Q1)
    return quat


def getQuatK():
    quat = qToFloat(rawQuatK, rotationVector_Q1)
    return quat


def getQuatReal():
    quat = qToFloat(rawQuatReal, rotationVector_Q1)
    return quat


def getQuatAccuracy():
    return quatAccuracy


def getMagAccuracy():
    return magAccuracy


# MAIN
try:
    beginSPI(22, 27, 4, 18, 3000000, 0)  # freq for Linux Kernel 1953000, but BNO080 can go up to 3000000
    enableMagnetometer(10)
    enableRotationVector(10)
    i=0
    while i < 100:
        dataAvailable()
        print("Raw Quaternion I : ", rawQuatI, " - Raw Quaternion J : ", rawQuatJ)
        time.sleep(0.010)
        i += 1
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
