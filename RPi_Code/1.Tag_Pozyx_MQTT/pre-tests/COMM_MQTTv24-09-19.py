import paho.mqtt.client as mqtt  # import required library
import time
from pynput import keyboard
import json


# call back method on esc_press
def on_press(key):
    print("Key press detected")
    if key == keyboard.Key.esc:
        print("Escape key")


def on_release(key):
    if key == keyboard.Key.esc:
        print("Escape Key Released")
        print("Exiting Program")
        exit()
    else:
        print("Key released")


# call back method on connection
def on_connect(client, userdata, flags, ec):
    print("on_connect called in")
    if ec == 0:
        print("Connection successful")
        client.connected_flag = True  # set flag
        print("Subscribing to TOPIC")
        client.subscribe(TOPIC)
        client.loop_start()  # so it checks periodically its sending and receiving buffers and process their content
    else:
        print("Connection failed. Error code = ", ec)
        client.failed_connection_flag = True
        print("stoping client.loop")
        client.loop_stop()
        print("Exiting program.")
        exit()


# call back method on disconnection
def on_disconnect(client, userdata, ec):
    client.loop_stop()  # stop checking send and receive buffers
    client.connected_flag = False
    client.disconnected_flag = True
    if ec == 0:
        print("Clean lost of connection with the broker ", broker_address, ". Error Code = ", ec)
        print("Exiting program")
    else:
        print("Connection lost with the broker ", broker_address, ". Error Code = ", ec)
        # test what logging.info does exactly
        #logging.info("Connection lost with the broker ", broker_address, ". Error Code = ", ec)
        print("Exiting program.")
    exit()


# call back method for messages
def on_message(client, userdata, message):
    client.message_received_flag = True
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)

    # handling json
    json_info_obj = json.loads(str(message.playload.decode("uft-8")))
    print("message received from tag_id :", json_info_obj.data.tagData.extras.tagId)


# INIT

# add flags in the class
mqtt.Client.connected_flag = False
mqtt.Client.disconnected_flag = False
mqtt.Client.failed_connection_flag = False
mqtt.Client.message_received_flag = False

# declarations
TOPIC = "tags"
counterMax = 0
broker_address = "172.30.4.44"  # define broker address
client = mqtt.Client("rasp_GI")  # instanciate the client with its name

# adding call back methods
client.on_connect = on_connect  # bind call back method
client.on_disconnect = on_disconnect
client.on_message = on_message

# MAIN - connection
client.loop_start()  # start a loop
print("Trying to connect to the broker ", broker_address)
client.connect(broker_address)  # connect to the specified broker
print("Waiting for connection")
while not client.connected_flag and not client.failed_connection_flag:  # waiting for co
    # just waiting for it to connect
    time.sleep(1)  # waits 1 second

# listening the keyboard for keypress
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

# while not input() == "exit()":
#    print("Exiting program")
#    exit()
