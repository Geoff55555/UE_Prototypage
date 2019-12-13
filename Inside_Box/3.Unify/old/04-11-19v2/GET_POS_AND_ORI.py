import Pozyx_MQTTvCONSOLE as pozyx
import BNO080_SPI as IMU

# -------------------------------------------------------------------
# -------------------------- POZYX TAG ------------------------------
# -------------------------------------------------------------------
print("[MAIN] Starting Tag Position Gathering")
myTag = pozyx.Tag("tags", "172.30.4.44", "rasp_GI")
myTag.start()

# -------------------------------------------------------------------
# --------------------------- BNO080 --------------------------------
# -------------------------------------------------------------------

# Pins config
# RECOMMANDED SETUP : What we want to set for default wiring
bcm_CSPin = 22
bcm_WAKPin = 27
bcm_INTPin = 4
bcm_RSTPin = 18
spiPortSpeed = 1953000  # 1953000 recommended for Linux Kernel, max 3000000 for BO080
spiPort = 0
report_delay_ms = 100  # may try 10ms

print("[MAIN] Starting BNO080")
myIMU = IMU.BNO080(bcm_CSPin, bcm_WAKPin, bcm_INTPin, bcm_RSTPin, spiPortSpeed, spiPort, report_delay_ms)
while repeat:
    if myIMU.calibrate(False):  # repeats until calibration complete
        repeat = False

try:
    # keep the program running
    while True:
        myIMU.get_data(100)


# -------------------------------------------------------------------
# ----------------------- END THE PROGRAM ---------------------------
# -------------------------------------------------------------------
# use CTRL + \ to quit in console
except KeyboardInterrupt: # never triggered, don't know why
    print("CTRL+/ pressed. Exiting Program")
    myTag.close()
    myIMU.close()
    exit()