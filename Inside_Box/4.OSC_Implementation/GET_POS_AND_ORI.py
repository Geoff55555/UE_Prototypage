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
quat = [0, 0, 0, 0]  # quaternion in case ori euler angles have conversion problems
ori = [0, 0, 0]
accel = [0, 0, 0]
tags_id = [0]


# -------------------------------------------------------------------
# ------------------------ REGISTER DATA ----------------------------
# -------------------------------------------------------------------
def new_data(user_tag_id):
    global coord, quat, ori, accel
    new_value = False
    if not(coord == pozyx_gateway.get_coord(user_tag_id)):
        coord = pozyx_gateway.get_coord(user_tag_id)
        new_value = True
    #if not (ori == myIMU.get_euler_ori()):
    #    ori = myIMU.get_euler_ori()
    #    new_value = True
    if not (quat == myIMU.get_quat_r_ijk()):
        quat = myIMU.get_quat_r_ijk()
        new_value = True
    if not (accel == myIMU.get_lin_accel()):
        accel = myIMU.get_lin_accel()
        new_value = True
    return new_value


# -------------------------------------------------------------------
# --------------------------- OSC COMM ------------------------------
# -------------------------------------------------------------------
# Start the system
osc_startup()

# Make client channels to send packets
#pc_ip = "192.168.0.205"
pc_ip = "172.30.40.40"
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
report_delay_ms = 1  # may try 10ms

print("[MAIN] Starting BNO080")
myIMU = IMU.BNO080(bcm_CSPin, bcm_WAKPin, bcm_INTPin, bcm_RSTPin, spiPortSpeed, spiPort, report_delay_ms)
repeat = True
print("[MAIN] Calibrating BNO080")
while repeat:
    if myIMU.calibrate(False):  # repeats until calibration complete
        repeat = False

myIMU.restart()
# works
#while True:
#    myIMU.get_latest_data()
#    None

try:
    # keep the program running
    while True:
        osc_process()
        myIMU.get_latest_data()
        if new_data(26920):  # if True there are new data
            print("send osc msg bundle")
            bundle = oscbuildparse.OSCBundle(oscbuildparse.OSC_IMMEDIATELY,
                      [oscbuildparse.OSCMessage("/test/posi/x", None, [coord[0]]),
                       oscbuildparse.OSCMessage("/test/posi/y", None, [coord[1]]),
                       oscbuildparse.OSCMessage("/test/posi/z", None, [coord[2]]),
                       #oscbuildparse.OSCMessage("/test/ori/x", None, [ori[0]]),
                       #oscbuildparse.OSCMessage("/test/ori/y", None, [ori[1]]),
                       #oscbuildparse.OSCMessage("/test/ori/z", None, [ori[2]]),
                       oscbuildparse.OSCMessage("/test/quat/r", None, [quat[0]]),
                       oscbuildparse.OSCMessage("/test/quat/i", None, [quat[1]]),
                       oscbuildparse.OSCMessage("/test/quat/j", None, [quat[2]]),
                       oscbuildparse.OSCMessage("/test/quat/k", None, [quat[3]]),
                       oscbuildparse.OSCMessage("/test/accel/x", None, [accel[0]]),
                       oscbuildparse.OSCMessage("/test/accel/y", None, [accel[1]]),
                       oscbuildparse.OSCMessage("/test/accel/z", None, [accel[2]])
                      ])
            osc_send(bundle, pc_client_name)
            print("sent")


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
