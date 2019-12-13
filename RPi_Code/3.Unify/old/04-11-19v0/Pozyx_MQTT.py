import paho.mqtt.client as mqtt  # import required library
import time
from pynput import keyboard
import json


class Tag:
    # attributes
    TOPIC = "None"
    broker_address = None
    client_name = None

    # function to get the data we want from the .json file received
    def get_tag_and_pos(raw_data):
        # raw data is a json file but with squared brackets at first and last char, let's remove them
        json_file = raw_data[1:-1]  # the negative index "-x" implies doing "len(raw_data)-x"
        # let's create a python dictonary from the json file
        py_dict = json.loads(json_file)
        # now we can get any info from the json_file but working with a py_dict
        data_to_print = 'Tag_ID = ' + str(py_dict["tagId"]) + ' --> Coord = ' + str(py_dict["data"]["coordinates"])
        return data_to_print

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
        else:
            print("Connection failed. Error code = ", ec)
            client.failed_connection_flag = True
            print("stoping client.loop")
            client.loop_stop()  # stop checking the call backs
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
            # logging.info("Connection lost with the broker ", broker_address, ". Error Code = ", ec)
            print("Exiting program.")
        exit()

        # call back method for messages

    def on_message(client, userdata, message):
        client.message_received_flag = True
        print("message topic =", message.topic)
        print("message qos =", message.qos)
        # handling json
        print(get_tag_and_pos(str(message.payload.decode("utf-8"))))

    # INIT
    def __init__(self, user_topic, user_broker_address, user_client_name):
        global TOPIC, broker_address, client, client_name  # to be able to write if necessary in those variables
        # add flags in the class for debugging purposes
        mqtt.Client.connected_flag = False
        mqtt.Client.disconnected_flag = False
        mqtt.Client.failed_connection_flag = False
        mqtt.Client.message_received_flag = False
        # declarations
        TOPIC = user_topic  # "tags"
        broker_address = user_broker_address  # "172.30.4.44"  # define broker address
        client_name = user_client_name
        client = mqtt.Client(client_name)  # instanciate the client with its name, i.e. "rasp_GI"
        # adding call back methods
        client.on_connect = on_connect  # bind call back method
        client.on_disconnect = on_disconnect
        client.on_message = on_message

    # MAIN - connection
    def start(self):
        global client  # to be able to write if necessary in those variables
        client.loop_start()  # start a loop like an update to check for call backs
        print("Trying to connect to the broker ", broker_address)
        client.connect(broker_address)  # connect to the specified broker
        print("Waiting for connection")

        # listening the keyboard for keypress (blocking but call backs are still launched)
        with keyboard.Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            listener.join()
