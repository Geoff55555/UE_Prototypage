import Pozyx_MQTTvCONSOLE as pozyx
import BNO080_SPI as IMU
import keyboard
# pay great attention to install keyboard with
# sudo pip3 install keyboard
# DON'T FORGER THE SUDO, IT WON'T WORK OTHERWISE

# -------------------------------------------------------------------
# --------------------- LOCAL DATA STORAGE --------------------------
# -------------------------------------------------------------------
coord = [0, 0, 0]
quat = [0, 0, 0, 0]


# -------------------------------------------------------------------
# ------------------------ REGISTER DATA ----------------------------
# -------------------------------------------------------------------
def update_data():
    global  quat, coord
    coord = pozyx_gateway.get_coord(26920)
    quat = myIMU.get_quat_r_ijk()

# -------------------------------------------------------------------
# -------------------------- SEND DATA ------------------------------
# -------------------------------------------------------------------
def send_data():
    update_data()
    return [coord, quat]

# -------------------------------------------------------------------
# ---------------------- SIMULATE REQUEST ---------------------------
# -------------------------------------------------------------------
# add callback method on key press event, see test_keypress for explanation
keyboard.on_press_key("g", lambda _:print("You pressed g!\nHere are the last values : ", myIMU.get_quat_r_ijk(),
                                          "\n", pozyx_gateway.get_coord(26920)))

# -------------------------------------------------------------------
# -------------------------- POZYX TAG ------------------------------
# -------------------------------------------------------------------
print("[MAIN] Starting Tag(s) Position Gathering from Pozyx Gateway")
pozyx_gateway = pozyx.Gateway("tags", "172.30.4.44", "rasp_GI")

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
repeat = False
while repeat:
    if myIMU.calibrate(False):  # repeats until calibration complete
        repeat = False

myIMU.continuous_data_update()

try:
    # keep the program running
    while True:
        None  # we're only using the callback of the keyboard to write down the data

# -------------------------------------------------------------------
# ----------------------- END THE PROGRAM ---------------------------
# -------------------------------------------------------------------
# use CTRL + \ to quit in console
except KeyboardInterrupt: # never triggered, don't know why
    print("CTRL+/ pressed. Exiting Program")
    myTag.close()
    myIMU.close()
    exit()
