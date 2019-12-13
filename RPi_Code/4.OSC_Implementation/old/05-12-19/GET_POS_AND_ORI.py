import Pozyx_MQTTvCONSOLE as pozyx
import BNO080_SPI as IMU
import keyboard
# pay great attention to install keyboard with
# sudo pip3 install keyboard
# DON'T FORGER THE SUDO, IT WON'T WORK OTHERWISE

#library for OSC comm
from osc4py3.as_eventloop import *
from osc4py3 import oscbuildparse

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
    print("HEEEREEE ", coord)
    #coord = [1, 2, 3]
    #while not myIMU.update_data():
    #    None  # waits for last data update
    quat = myIMU.get_quat_r_ijk()

# -------------------------------------------------------------------
# -------------------------- SEND DATA ------------------------------
# -------------------------------------------------------------------
# DEPRECIATED WITH PUREDATA
def get_latest_data(user_tag_id):
    update_data(user_tag_id)
    return [coord, quat]  # puredata doens't like double array


def get_ori():
    return quat


def get_posi():
    return coord

# -------------------------------------------------------------------
# --------------------------- OSC COMM ------------------------------
# -------------------------------------------------------------------
# Start the system
osc_startup()

# Make client channels to send packets
#pc_ip = "192.168.0.205"
pc_ip = "172.30.40.18"
pc_port = 9001
pc_client_name = "pc_gi"
osc_udp_client(pc_ip, pc_port, pc_client_name)

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
report_delay_ms = 20  # may try 10ms

print("[MAIN] Starting BNO080")
myIMU = IMU.BNO080(bcm_CSPin, bcm_WAKPin, bcm_INTPin, bcm_RSTPin, spiPortSpeed, spiPort, report_delay_ms)
repeat = False
print("[MAIN] Calibrating BNO080")
while repeat:
    if myIMU.calibrate(False):  # repeats until calibration complete
        repeat = False

myIMU.restart()
# works
#while True:
    #myIMU.up()
    #None

try:
    i = 0
    previous = 0
    # keep the program running
    while True:
        osc_process()
        i += 1
        myIMU.up()
        update_data(26920)
#        current_data = get_latest_data(26920)
#        if previous != current_data:
#        print("yo", current_data)
        print("send osc msg bundle")
        bundle = oscbuildparse.OSCBundle(oscbuildparse.OSC_IMMEDIATELY,
                  [oscbuildparse.OSCMessage("/test/posi/x", None, [coord[0]]),
                   oscbuildparse.OSCMessage("/test/posi/y", None, [coord[1]]),
                   oscbuildparse.OSCMessage("/test/posi/z", None, [coord[2]]),
                   oscbuildparse.OSCMessage("/test/ori/r", None, [quat[0]]),
                   oscbuildparse.OSCMessage("/test/ori/i", None, [quat[1]]),
                   oscbuildparse.OSCMessage("/test/ori/j", None, [quat[2]])
                  ])
        osc_send(bundle, pc_client_name)
        print("sent")
#        previous = current_data
        # doesn't work with if
#        if keyboard.is_pressed("g"):
#            print("You pressed g !\nHere are the latest data : \n",
#                  "coord then quat : ", get_latest_data(26920))
        #
        #None  # we're only using the callback of the keyboard to write down the data

# -------------------------------------------------------------------
# ----------------------- END THE PROGRAM ---------------------------
# -------------------------------------------------------------------
# use CTRL + \ to quit in console
except KeyboardInterrupt: # never triggered, don't know why
    print("[MAIN] CTRL+/ or CTRL+C pressed. Exiting Program")
    print("[MAIN]    Closing OSC Communication")
    osc_terminate()
    print("[MAIN]    Closing BNO080 connection")
    myIMU.close()
    print("[MAIN]    Closing the pozyx gateway connection")
    pozyx_gateway.close()  # calls exit() and ends the program
