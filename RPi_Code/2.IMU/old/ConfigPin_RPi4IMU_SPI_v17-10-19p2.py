# Author : Geoffrey ISHIMARU - MA1 ELN - ISIB
# References : [1] BNO080 Datasheet from Hillcrest labs, [2] Sparkfun_BNO080.cpp updated by Guillaume Villee, [3] Sparkfun_BNO080.h, [4] Reference Manual from Hillcrestlabs
#              [5] IMUManager.cpp modified by Guillaume Villee

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
debugPrint = True
timeStamp = 0  #32-bit value
firstPackReceived = False

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

COMMAND_DCD = 6  # [3] L91
COMMAND_ME_CALIBRATE = 7  # [3] L92

CALIBRATE_ACCEL = 0  # [3] from L97
CALIBRATE_GYRO = 1
CALIBRATE_MAG = 2
CALIBRATE_PLANAR_ACCEL = 3
CALIBRATE_ACCEL_GYRO_MAG = 4
CALIBRATE_STOP = 5

channel = ["Command", "Executable", "Control", "Sensor-report", "Wake-report", "Gyro-vector"]  # Meaning got from [2] L1083

# - Communication Data Storage
shtpHeader = []
shtpData = []
dataLength = 0  # length of shtpData
seqNbr = [0, 0, 0, 0, 0, 0]  # There are 6 comm channels. Each channel has its own seqNum [3]L198

# - Raw sensor values
rawAccelX, rawAccelY, rawAccelZ, accelAccuracy = None, None, None, None  # [3] L214
rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy = None, None, None, None
rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy = None, None, None, None
rawMagX, rawMagY, rawMagZ, magAccuracy = None, None, None, None
rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy = None, None, None, None, None, None
stepCount = None
timeStamp = None
stabilityClassifier = None
activityClassifier = None

_activityConfidence = []  # Array that store the confidences of the 9 possible activities
for i in range(9):
    _activityConfidence.append(None)

rotationVector_Q1 = 14  # [3]L227 These Q values are defined in the datasheet but can also be obtained by querying the meta data records
accelerometer_Q1 = 8
linear_accelerometer_Q1 = 8
gyro_Q1 = 9
magnetometer_Q1 = 4

# - Calibration
calibrationStatus = None  # Byte R0 of ME Calibration Response
isCalibrating = False
calibrated = False
calPrecisionRate = 0
commandSequenceNumber = 0


# -------------------------------------------------------------------
# --------------------- COMMUNICATION INIT --------------------------
# -------------------------------------------------------------------

def beginSPI(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_spiPortSpeed, user_spiPort):  # [2]L33
    # We want the global variable pins declared above
    global _cs, _wake, _int, _rst, spi, shtpData

    # Get user settings
    if debugPrint: print("[BNO080] Setting up SPI communication")
    _spiPort = user_spiPort
    _spiPortSpeed = user_spiPortSpeed  # up to 3MHz allowed by BNO but RPi offers 3.9MHz or 1.953MHz (because of clock divider values)

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
        spi.open(_spiPort, 0) # first param = 0 is the bus (corresponding to the spiPort), second param = 0* is the device
        # *set to 0 if use of 10 and 11 -MOSI and SCLK- GPIO (SPI0), set to 1 if 20 and 21 (SPI1)
        if debugPrint: print("[BNO080] Setting SPI communication parameters")
        spi.mode = 0b11 # https://pypi.org/project/spidev
        print("[BNO080] SPI mode set to : " + str(spi.mode))
        spi.max_speed_hz = _spiPortSpeed
        print("[BNO080] SPI Freq [Hz] set to : " + str(spi.max_speed_hz))

        # Comment from sparkfun : " At system startup, the hub must send its full advertisement message (see 5.2 and 5.3)
        # to the host. It must not send any other data until this step is complete.
        # When BNO080 first boots it broadcasts big startup packet. Read it and dump it."
        waitForPinResponse(_int) # Wait for assertion of INT before reading Init response

        if debugPrint: print("[BNO080] Very first advertisement packet awaited -not relevant-")
        receiveSPIPacket()        
        # print(shtpData)  # just to check if the global shtpData has been used

        # [2] L83 "BNO080 will then transmit an unsolicited Initialize Response (see 6.4.5.2)"
        waitForPinResponse(_int)  # [2] wait for assertion of INT before reading Init response
        if debugPrint: print("[BNO080] Unsolicited Init awaited -not relevant-")  # [4] p. 44 - 6.4.5.2
        receiveSPIPacket()
        if shtpData[0] == 0xF1 and shtpData[2] == 0x84 and shtpData[5] == 0:
            print("[BNO080] is well initialized according to Ref Manual p.44 6.4.5.2")
        else:
            print("[BNO080] WARNING ! INIT PROBLEM !!")
            return False
        # if debugPrint: print(shtpData)  # just to check if the global shtpData has been used
        
        # ADD THE COMMENTED LINES 90 TO 97 from [2]!!
        #print("RPi is sending a solicited init command packet")
        #shtpData[0] = SHTP_REPORT_COMMAND_REQUEST
        #shtpData[1] = 0
        #sendSPIPacket(channel.index("Control"), 2)
        #print("RPi is waiting for sollicited response")
        #waitForPinResponse(_int)
        #receiveSPIPacket()

        # Check communication with the device [2]L100 / [4] Product ID Request p.37 Figure 35 and Check the response p.38
        shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST  # Request the product ID and reset info
        shtpData[1] = 0                               # Reserved
        if debugPrint: print("[BNO080] RPi changing shtpData to send test packet")
        if debugPrint: print("[BNO080] Check bytes 0 = 0xF9 and byte 1 = 0 :", shtpData)  # just to check if the global shtpData has been used

        # Transmit packet on channel 2 (= control), 2 bytes [2]L103
        if debugPrint: print("[BNO080] Test Packet ready ! Launching sendSPIPacket() method")
        sendSPIPacket(channel.index("Control"), 2)

        # Now wait for BNO080 response
        waitForPinResponse(_int)
        # Receiving the response / [4] Check the response p38: bit0 = 0xF8 = SHTP_REPORT_PRODUCT_ID_RESPONSE
        if debugPrint: print("[BNO080] Response to Test Packet awaited")
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

    for counter in range(10000):
        if GPIO.input(input) == GPIO.LOW:
            if debugPrint:
                if input == 4:
                    print("[BNO080] ready!")
                    print(".") # lighten log
                else:
                    print("[BNO080] RPi Pin BCM nbr "+ str(input)+ " connected!")
            return True
        time.sleep(0.001) # waits 1ms (under that 1ms it won't succeed to connect)
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
    global shtpData, shtpHeader, dataLength, firstPackReceived  # because we want to use the global shtpData defined in the first lines of this code
    if GPIO.input(_int) == GPIO.HIGH:
        print("[BNO080] Data not available")
        return False
    
    if debugPrint: print("[BNO080] Data received!")
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
    shtpData.clear()
    shtpData = spi.readbytes(dataLength)
    if debugPrint: print("[BNO080] Data successfully stored in shtpData")
    # check if the packet is the one from init
    if shtpData == [0, 1, 4, 0, 0, 0, 0, 128, 6, 49, 46, 48, 46, 48, 0, 2, 2, 0, 1, 3, 2, 255, 127,
                    4, 2, 0, 1, 5, 2, 255, 127, 8, 5, 83, 72, 84, 80, 0, 6, 1, 0, 9, 8, 99, 111, 110,
                    116, 114, 111, 108, 0, 1, 4, 1, 0, 0, 0, 8, 11, 101, 120, 101, 99, 117, 116, 97, 98,
                    108, 101, 0, 6, 1, 1, 9, 7, 100, 101, 118, 105, 99, 101, 0, 1, 4, 2, 0, 0, 0, 8, 10,
                    115, 101, 110, 115, 111, 114, 104, 117, 98, 0, 6, 1, 2, 9, 8, 99, 111, 110, 116, 114,
                    111, 108, 0, 6, 1, 3, 9, 12, 105, 110, 112, 117, 116, 78, 111, 114, 109, 97, 108, 0, 7,
                    1, 4, 9, 10, 105, 110, 112, 117, 116, 87, 97, 107, 101, 0, 6, 1, 5, 9, 12, 105, 110, 112,
                    117, 116, 71, 121, 114, 111, 82, 118, 0, 128, 6, 49, 46, 49, 46, 48, 0, 129, 100, 248, 16,
                    245, 4, 243, 16, 241, 16, 251, 5, 250, 5, 252, 17, 239, 2, 1, 10, 2, 10, 3, 10, 4, 10, 5, 14,
                    6, 10, 7, 16, 8, 12, 9, 14, 10, 8, 11, 8, 12, 6, 13, 6, 14, 6, 15, 16, 16, 5, 17, 12, 18, 6,
                    19, 6, 20, 16, 21, 16, 22, 16, 23, 0, 24, 8, 25, 6, 26, 0, 27, 0, 28, 6, 29, 0, 30, 16, 31, 0,
                    32, 0, 33, 0, 34, 0, 35, 0, 36, 0, 37, 0, 38, 0, 39, 0, 40, 14, 41, 12, 42, 14]:
        firstPackReceived = True
        print("It was the very first advertisement packet we received (again)")
    else:
        firstPackReceived = False
    if debugPrint and not firstPackReceived:
        printHeader()
        print(shtpHeader, dataLength, shtpData)
    # Finishing receive data from BNO080 - Release BNO080
    GPIO.output(_cs, GPIO.HIGH)
    # Done !
    return True


def sendSPIPacket(channelNbr, data_length):  # [2]L1017
    # We want to use global variables to be able to write and save the changes about them
    global seqNbr
    # packet length contains header (4 bytes) + data
    packetLength = data_length + 4
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
    elif packetLength >= 256 and packetLength < 65535:
        headerBuffer[0] = packetLength % 256  # packet length LSB
        headerBuffer[1] = packetLength // 256  # packet length MSB
    else:
        print("           !!!!           ")
        print("PACKET TO SEND IS TOO LONG")
        print("           !!!!           ")

    headerBuffer[2] = channelNbr
    headerBuffer[3] = seqNbr[channelNbr]
    seqNbr[channelNbr] =  seqNbr[channelNbr] + 1

    # 2. Send the header to BNO080
    if debugPrint: print("[BNO080] RPi Sending headerBuffer : " + str(headerBuffer))
    spi.writebytes(headerBuffer)

    # 3. Prepare user's data packet
    buffer = []
    for i in range(data_length):
        buffer.append(shtpData[i])
    # 4. Send user's data to BNO080
    if debugPrint: print("[BNO080] RPi Sending data : " + str(buffer))
    spi.writebytes(buffer)

    # Release BNO080
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
                print(".")
                if (shtpHeader[2] == channel.index("Sensor-report") and shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP):
                    print("[BNO080] has data available to send from its sensors !")
                    parseInputReport()
                    return True
                elif shtpHeader[2] == channel.index("Control") or shtpHeader[2] == channel.index("Command"):
                    print("[BNO080] sent responses about the RPi commands (status, etc.) - NO SENSOR DATA")
                    # what is said by [2] L151 but I got to add "Command" to manage the calibration reports
                    parseCommandReport()
                    return True
                else:
                    print("channel received = ", channel[shtpHeader[2]])
            except IndexError:
                print("[BNO080] Channel number is not recognized")
    return False


# Given a sensor's report ID, this tells the BNO080 to begin reporting the values
# Also sets the specific config word. Useful for personal activity classifier
def setFeatureCommandSpecConf(reportID, timeBetweenReports, specificConfig):  # [2] L794 -- Time in milliseconds
    microsBetweenReports = timeBetweenReports * 1000  # (32-bit nbr)
    # Making shtpData array at right length to avoid index out of range
    shtpData.clear()
    for i in range(17):  # 0 to 18 not included
        shtpData.append(0)
    # Preparing packet
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND     # set feature command [4]p.55+56
    shtpData[1] = reportID                            # Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0                                   # Feature flags
    shtpData[3] = 0                                   # Change sensitivity (LSB)
    shtpData[4] = 0                                   # Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports & 0xFF)                      # Report Interval (LSB from 32-bit) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports & 0xFF00) // pow(256,1)      # Report Interval
    shtpData[7] = (microsBetweenReports & 0xFF0000) // pow(256,2)    # Report Interval
    shtpData[8] = (microsBetweenReports & 0xFF000000) // pow(256,3)  # Report Interval (MSB)
    shtpData[9] = 0                                   # Batch Interval (LSB)
    shtpData[10] = 0                                  # Batch Interval
    shtpData[11] = 0                                  # Batch Interval
    shtpData[12] = 0                                  # Batch Interval (MSB)
    shtpData[13] = (specificConfig & 0xFF)                            # Sensor-specific config (LSB)
    shtpData[14] = (specificConfig & 0xFF00) // 256                   # Sensor-specific config
    shtpData[15] = (specificConfig & 0xFF0000) // pow(256,2)          # Sensor-specific config
    shtpData[16] = (specificConfig & 0xFF000000) // pow(256,3)        # Sensor-specific config (MSB)
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
    print("Enabling Magnetometer (with time between reports = ", timeBetweenReports, ")")
    setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports)


# Sends the packet to enable the rotation vector --> return a quaternion = magnetometer + accelerometer + gyroscope /!\ enable magnetometer first !
def enableRotationVector(timeBetweenReports):  #[2] L689
    print("Enabling Rotation vector (with time between reports = ", timeBetweenReports,")")
    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports)


def enableGameRotationVector(timeBetweenReports):
    print("Enabling Game Rotation Vector (with time between reports = ", timeBetweenReports,")")
    setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports)
    
    
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
    
    try:
        timeStamp = shtpData[4]*pow(256,3) + shtpData[3]*pow(256,2) + shtpData[2]*256 + shtpData[1]
        
        print(shtpData[5 + 2])
        status = shtpData[5 + 2] & 3  # equivalent of binary operation shtpData[5 + 5] & 0x03
        print("Status : ", status)        
        data1 = shtpData[5 + 5]*256 + shtpData[5 + 4]
        data2 = shtpData[5 + 7]*256 + shtpData[5 + 6]
        data3 = shtpData[5 + 9]*256 + shtpData[5 + 8]
        data4 = 0
        data5 = 0

        if dataLength - 5 > 10:  # [2] L230 originally dataLength - 5 > 9 but index out of range ! (when length = 15)
            data4 = shtpData[5 + 11]*256 + shtpData[5 + 10]
        if dataLength - 5 > 12:  # [2] L234 originally dataLength - 5 > 11 but index out of range possible !
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
            # convert quat ? [5] L360
            print("[BNO080] quatAccuracy = ", status)
            print("[BNO080] QUATERNION R-IJK = [", getQuatReal(), " - ", getQuatI(), " , ", getQuatJ(), " , ", getQuatK(), " ]")

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
                print("[BNO080] Calibration status = ", calibrationStatus)

        else:
            print("[BNO080] Sensor Report ID not handled")

        return True
    except IndexError as e:
        print("[BNO080] Probably a Package length error : ", e)


def parseCommandReport():
    global calibrationStatus
    if shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE:
        print("[BNO080] Report Command Response")
        # The BNO080 responds with this report to command requests.It's up to use to remember which command we issued.
        command = shtpData[2]  # This is the Command byte of the response
        print("[BNO080] Command byte is ", command)

        if command == COMMAND_ME_CALIBRATE:
            print("[BNO080] sent calibrate command request")
            calibrationStatus = shtpData[5 + 0]  # R0 - Status (0 = success, non-zero = fail)
            print("[BNO080] Calibration answer is : ", calibrationStatus, " (0 = SUCCESS otherwise FAIL")
        else:
            print("[BNO080] command report not handled")
    else:
        # This sensor report ID is unhandled.
        # See [4] to add additional feature reports as needed
        print("[BNO080] Sensor report ID = ", shtpData[0], "is not handled")

    # TODO additional feature reports may be strung together. Parse them all.

# -------------------------------------------------------------------
# ---------------------- GET EVERYTHING -----------------------------
# -------------------------------------------------------------------

# Given a register value and a Q point, convert to float [2] L677
def qToFloat(fixedPointValue, qPoint):
    qFloat = 0
    if not fixedPointValue == None:
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


# -------------------------------------------------------------------
# ------------------------ CALIBRATION ------------------------------
# -------------------------------------------------------------------

# Generic calibration function to call for calibration. The others are included in this one
def calibrateIMU():  # [5] L108 - adapted/modified
    print("[BNO080] CalibrateIMU() called")
    global isCalibrating, calibrated
    if isCalibrating == False:
        isCalibrating = True
        calibrated = False
        # try to calibrate and if successful, save that calibration to BNO080
        count = 0
        trials = 2
        # 4 next lines already done in StartIMU()
        # Enable Game Rotation Vector output --> mix accel + gyro
        # enableRotationVector(5)  # Send data update every 5ms
        # Enable Magnetic Field output -> only magneto
        # enableMagnetometer(5)  # Send data update every 5ms
        while not calibrated:
            # send the command to calibrate all sensors
            calibrateAll()
            # check if has been calibrated
            checkCalibration()  # checks the calibration and updates the calPrecisionRate
            # if the precision rate is high enough, IMU is taken as already calibrated
            if calPrecisionRate >= 0.8:
                print("[BNO080] Percentage good enough to save.")
                print("[BNO080] Saving calibration to the IMU...")
                # Try to save to the BNO080, after 100 fails, stop
                count = 0
                while count <= trials:
                    saveCalibration()
                    time.sleep(0.001)  # Waits 1ms before checking for the BNO080 to have the time to save
                    # ask the BNO080 if the calibration has been successfully saved
                    requestCalibrationStatus()
                    if dataAvailable():
                        if calibrationComplete():
                            print("[BNO080] Calibration successfully saved to BNO080 !")
                            calibrated = True
                            print("[BNO080] Ending Calibration")
                            endCalibration()
                            return True
                print("[BNO080] failed to store calibration data")
            else:
                print("[BNO080] Precision rate not high enough (<80%) to be stored")

    isCalibrating = False


def calibrateAll():  # [5] L118
    sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG)


# [2] L844
# This tells the BNO080 to begin calibrating
# See [4] p.50 and 1000-4044 calibration doc
def sendCalibrateCommand(thingToCalibrate):
    global shtpData, calibrationStatus
    # Clear the part 3 to 11 section of shtpData array
    shtpData.clear()
    for i in range(12):  # 0 to 12 not included
        shtpData.append(0)
    # for i in range(3, 12):  # 3 included to 12 not included
    #    shtpData[i] = 0
    
    # Check what to calibrate and modify the command accordingly
    if thingToCalibrate == CALIBRATE_ACCEL:
        shtpData[3] = 1
    elif thingToCalibrate == CALIBRATE_GYRO:
        shtpData[4] = 1
    elif thingToCalibrate == CALIBRATE_MAG:
        shtpData[5] = 1
    elif thingToCalibrate == CALIBRATE_PLANAR_ACCEL:
        shtpData[7] = 1
    elif thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG:
        shtpData[3] = 1
        shtpData[4] = 1
        shtpData[5] = 1
    elif thingToCalibrate == CALIBRATE_STOP:
        # Do nothing, bytes are set to 0
        print("[BNO080] Send \"CALIBRATE_STOP\" command")

    # Make the internal calStatus variable non-zero (operation failed) so that the user can test while we wait
    calibrationStatus = 1

    # using this shtpData packet, send a command
    print("[BNO080] Sending calibration request")
    sendCommand(COMMAND_ME_CALIBRATE)


# Generic check : checks the calibration and updates the calPrecisionRate
def checkCalibration():  # [5]L70 + L108 + L311 mix
    global calibrated, calPrecisionRate
    # Enable Game Rotation Vector output --> mix accel + gyro
    # enableRotationVector(5)  # Send data update every 5ms
    # Enable Magnetic Field output -> only magneto
    # enableMagnetometer(5)  # Send data update every 5ms
    # Check the first one hundred values
    count = 0
    goodData = 0
    while count < 10:
        # before calling checkCalibration, calibrateAll() should be called
        # therefore, if data available, they will contain quatAccuracy OR magAccuracy
        # error in [5] L160 --> we only receive either quatAccuracy in one packet OR magAccuracy
        # i.e : f they get quatAccuracy, they reuse last magAccuracy stored...
        # --> So if last nbrs are not updated, they consider the last nbrs are still ok
        print("[BNO080] Checking calibration")
        if dataAvailable():  # updates the values if data available (and there will be since calibrateAll() sent calibrate cmd)
            print("[BNO080] quatAcc = ", quatAccuracy, ", magAcc = ", magAccuracy)
            if quatAccuracy == 3 and magAccuracy == 3:  # if True, high accuracy. We need it for both in a row (so 2 following packets received contained 1 quatAcc =3 and the next magAcc = 3 or in opposite order)
                goodData += 1
                print("[BNO080] Good Data = ", goodData, " on ", count, " tests")
        count += 1
        time.sleep(0.1)  # sleeps for 1s

    calPrecisionRate = goodData / count
    print("[BNO080] percentage of data accuracy is : ", calPrecisionRate*100, " %")


# [2] L910
# This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
# See [4] p.49 and the 1000-4044 calibration doc
def saveCalibration():
    global shtpData
    # Clear the part 3 to 11 section of shtpData array
    for i in range(3, 12):  # 3 included to 12 not included
        shtpData[i] = 0
    # Using this shtpData packet, send a command
    sendCommand(COMMAND_DCD);  # Save DCD command


# [2] L887
# Request ME Calibration Status from BNO080
# See [4]p. 51
def requestCalibrationStatus():
    global shtpData
    # Clear the part 3 to 11 section of shtpData array
    for i in range(3, 12):  # 3 included to 12 not included
        shtpData[i] = 0
    shtpData[6] = 0x01  # P3 - 0x01 - Subcommand: Get ME Calibration
    # Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);  # Save DCD command


# [2] L777
# See [4] p.51 - ME Calibration Response
# Byte 5 is parsed during the readPacket and stored in calibrationStatus
def calibrationComplete():
    if calibrationStatus == 0:
        return True
    return False


def endCalibration():  # [2] L772
    sendCalibrateCommand(CALIBRATE_STOP)


# -------------------------------------------------------------------
# -------------------------- COMMAND --------------------------------
# -------------------------------------------------------------------

# Tell the sensor to do a command [2]L820
# See [4] p.41 6.3.8, Command Request
# The caller is expected to set P0 through P8 prior to calling
def sendCommand(command):
    global shtpData, commandSequenceNumber
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST
    shtpData[1] = commandSequenceNumber
    commandSequenceNumber += 1
    shtpData[2] = command

    # print("test ", shtpData)
    # Transmit packet on channel 2, 12 bytes
    sendSPIPacket(channel.index("Control"), 12)


# -------------------------------------------------------------------
# ------------------------ START/RESET ------------------------------
# -------------------------------------------------------------------

def StartIMU(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_bcm_spiPortSpeed, user_bcm_spiPort):
    count = 0
    while beginSPI(user_bcm_CSPin, user_bcm_WAKPin, user_bcm_INTPin, user_bcm_RSTPin, user_bcm_spiPortSpeed, user_bcm_spiPort) == False and count < 100:  # freq for Linux Kernel 1953000, but BNO080 can go up to 3000000
        print("[BNO080] over SPI not detected. Restart #", count + 1)
        count += 1
    print(".")
    print("Asking IMU to send data...")
    # Enable Game Rotation Vector output --> mix accel + gyro
    enableGameRotationVector(100)  # Send data update every 100ms
    # Enable Rotation Vector
    # enableRotationVector(100)  # Send data update every 100ms
    # Enable Magnetic Field output -> only magneto
    #enableMagnetometer(100)  # Send data update every 100ms


# [2] L635
# Send command to reset IC
# Read all advertisement packets from sensor
# The sensor has been seen to reset twice if we attempt too much too quickly
# This seems to work reliably
def softReset():
    global shtpData
    print("[BNO080] SOFT RESET")
    resetOk = False
    try:
        shtpData[0] = 1  # Reset
    except IndexError:
        shtpData.clear()
        shtpData.append(1)

    # Attempt to start communication with sensor
    sendSPIPacket(channel.index("Executable"), 1)  # Transmit packet on channel 1, 1 byte
    # Read all incoming data and flush it
    time.sleep(1)
    while receiveSPIPacket() == True:
        # Do nothing
        print("[BNO080] Communication re-established with BNO080 after reset. Flushing response")
        resetOk = True
    time.sleep(0.05)
    while receiveSPIPacket() == True:
        # Do nothing
        print("[BNO080] reset twice because softReset() sent to quickly. Don't worry, case managed.")
        resetOk = True
    if not resetOk:
        print("[BNO080] SOFT RESET FAILED")


# -------------------------------------------------------------------
# ------------------------- CALLBACKS -------------------------------
# -------------------------------------------------------------------

# https://raspi.tv/2013/how-to-use-interrupts-with-python-on-the-raspberry-pi-and-rpi-gpio-part-3

def INT_callback(input):
    print("[BNO080] Interrupt triggered")
    dataAvailable()
            

# -------------------------------------------------------------------
# --------------------------- MAIN ----------------------------------
# -------------------------------------------------------------------

try:
    bcm_CSPin = 22
    bcm_WAKPin = 27
    bcm_INTPin = 4
    bcm_RSTPin = 18
    spiPortSpeed = 1953000  # 1953000 recommended for Linux Kernel, max 3000000 for BO080
    spiPort = 0
    
    StartIMU(bcm_CSPin, bcm_WAKPin, bcm_INTPin, bcm_RSTPin, spiPortSpeed, spiPort)

    # add callback method on INT pin, bounetime = time [ms] after an INT that GPIO will ignore new INT
    GPIO.add_event_detect(bcm_INTPin, GPIO.FALLING, callback=INT_callback, bouncetime=100)
    
    # calibrate
    #print("-------------------CALIBRATION------------------")
    #calibrateIMU()
    # to change de vector to send (calibration uses another vector) [5]L254
    #softReset()
    
    # restart
    #StartIMU(bcm_CSPin, bcm_WAKPin, bcm_INTPin, bcm_RSTPin, spiPortSpeed, spiPort)
    while shtpHeader[3] < 50:
        time.sleep(0.001)
    # Just to make it wait with no particular process running
    #GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    #GPIO.wait_for_edge(24, GPIO.RISING)
    
    #lookForData = True
    #nbrNoData = 0
    #while lookForData:
    #    if dataAvailable():
    #        # convert quat ? [5] L360
    #        print("QUATERNION R-IJK = [", getQuatReal(), " - ", getQuatI(), " , ", getQuatJ(), " , ", getQuatK(), " ]")
    #    else:
    #        nbrNoData += 1
    #        if nbrNoData == 50:
    #            lookForData = False
    #            print("50x no Data. Consider IMU lost")
    #    time.sleep(0.05)

    print("Everything went SMOOTH")
except Exception as e:
    print("An error occurred. Error message : " + str(e))
finally:
    print('.')  # lighten log
    print("Closing SPI communication")
    spi.close()
    print("GPIO remove callbacks")
    GPIO.remove_event_detect()
    print("GPIO cleanup")
    GPIO.cleanup()

    print("Exiting Program")
