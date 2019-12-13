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
tags_id = [0]


# -------------------------------------------------------------------
# ------------------------ REGISTER DATA ----------------------------
# -------------------------------------------------------------------
def update_data(user_tag_id):
    global  quat, coord
    coord = pozyx_gateway.get_coord(user_tag_id)
    while not myIMU.update_data():
        None  # waits for last data update
    quat = myIMU.get_quat_r_ijk()

# -------------------------------------------------------------------
# -------------------------- SEND DATA ------------------------------
# -------------------------------------------------------------------
def get_latest_data(user_tag_id):
    update_data(user_tag_id)
    return [coord, quat]

# -------------------------------------------------------------------
# ---------------------- SIMULATE REQUEST ---------------------------
# -------------------------------------------------------------------
# add callback method on key press event, see test_keypress for explanation
#keyboard.on_press_key("g", lambda _:print("You pressed g!\nHere are the last values :\n",
#                                              "coord : ", pozyx_gateway.get_coord(26920), "\n",
#                                              "quat :  ", myIMU.get_quat_r_ijk()))
#keyboard.on_press_key("g", lambda _:send_data(tags_id[0]))

# -------------------------------------------------------------------
# -------------------------- POZYX TAG ------------------------------
# -------------------------------------------------------------------
print("[MAIN] Starting Tag(s) Position Gathering from Pozyx Gateway")
pozyx_gateway = pozyx.Gateway("tags", "172.30.4.44", "rasp_GI")
tags_id = [26920]  # when only one tag but could be a list with more tags

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
repeat = True
print("[MAIN] Calibrating BNO080")
while repeat:
    if myIMU.calibrate(False):  # repeats until calibration complete
        repeat = False

print("[MAIN] Waiting for you to ask data (press g)")
try:
    # keep the program running
    while True:
        if keyboard.is_pressed("g"):
            print("You pressed g !\nHere are the latest data : \n",
                  "coord then quat : ", get_latest_data(26920))
        #None  # we're only using the callback of the keyboard to write down the data

# -------------------------------------------------------------------
# ----------------------- END THE PROGRAM ---------------------------
# -------------------------------------------------------------------
# use CTRL + \ to quit in console
except KeyboardInterrupt: # never triggered, don't know why
    print("[MAIN] CTRL+/ or CTRL+C pressed. Exiting Program")
    print("[MAIN]    Closing BNO080 connection")
    myIMU.close()
    print("[MAIN]    Closing the pozyx gateway connection")
    pozyx_gateway.close()  # calls exit() and ends the program
