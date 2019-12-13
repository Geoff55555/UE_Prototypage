from osc4py3.as_eventloop import *
from osc4py3 import oscbuildparse

# Start the system
osc_startup()

# Make client channels to send packets
pc_ip = "172.30.40.40"
pc_port = 9001
pc_client_name = "pc_gi"
osc_udp_client(pc_ip, pc_port, pc_client_name)

# Build a message with autodetection of data types and send it
msg_posi = oscbuildparse.OSCMessage("/test/posi", None, ["text", 62, 8.9])
# msg_ori = oscbuildparse.OSCMessage("/test/ori", None, [33.3, 12.12, 62, 8.9])
# osc_send([msg_posi, msg_ori], pc_client_name)

osc_send(msg_posi, pc_client_name)

repeat = False
i = 0
while repeat:
    osc_process()
    i += 1
    if i >= 20:
        repeat = True
    
print("finish")
osc_terminate()