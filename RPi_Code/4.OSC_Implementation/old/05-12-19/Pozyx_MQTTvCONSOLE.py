import paho.mqtt.client as mqtt  # import required library
# in Linux use cmd : sudo pip3 install paho-mqtt
# don't forget the sudo !!!
import time
import json

# debug
debug_print = False


def enable_debugging():
    global debugPrint
    debug_print = True


# definitions
# variables
tags_id = []
tags_coord = []  # link between the 2 arrays is that we'll be using the same index for both to refer to the same tag


# function to get the data we want from the .json file received
def get_id_and_coord(raw_data):
    # raw data is a json file but with squared brackets at first and last char, let's remove them
    json_file = raw_data[1:-1]  # the negative index "-x" implies doing "len(raw_data)-x"
    # let's create a python dictonary from the json file
    py_dict = json.loads(json_file)
    # now we can get any info from the json_file but working with a py_dict
    id = int(py_dict["tagId"])
    if debug_print: print("[TAG] ID set to ", id)
    #print("TESSSSSST ", py_dict["data"]["coordinates"]["x"])
    coord = [0,0,0]
    coord[0] = py_dict["data"]["coordinates"]["x"]  # to be sure we have int
    coord[1] = py_dict["data"]["coordinates"]["y"]
    coord[2] = py_dict["data"]["coordinates"]["z"]
    try:
        id_current_tag_received_index = tags_id.index(id)  # check if this id has been already received
    except ValueError:  # may be ValueError (case in Windows PyCharm)
        tags_id.append(id)  # if not, we add the id of the tag in the tags_id list
        tags_coord.append([0, 0, 0])  # immediatly after, we also create a new array in tags_coord array so the indexes
        # match in both arrays refering to the same tag
        id_current_tag_received_index = tags_id.index(id)
    tags_coord[id_current_tag_received_index] = coord  # update the coord of the tag (id=x) at the same index as the one
    if debug_print: print("[TAG] Updated tags arrays (id then coord) :", tags_id, tags_coord)
    # corresponding in tags_id
    data_to_print = '[TAG] Tag_ID = ' + id + ' --> Coord = ' + str(coord)
    return data_to_print


# call back method on connection
def on_connect(client, userdata, flags, ec):
    print("[TAG] on_connect called in")
    if ec == 0:
        print("[TAG] Connection successful")
        client.connected_flag = True  # set flag
        print("[TAG] Subscribing to TOPIC")
        client.subscribe(TOPIC)
        print("[TAG] Subscribed")
    else:
        print("[TAG] Connection failed. Error code = ", ec)
        client.failed_connection_flag = True
        print("[TAG] Stoping client.loop")
        client.loop_stop()  # stop checking the call backs
        print("[TAG] Exiting program.")
        exit()


# call back method on disconnection
def on_disconnect(client, userdata, ec):
    client.loop_stop()  # stop checking send and receive buffers
    client.connected_flag = False
    client.disconnected_flag = True
    if ec == 0:
        print("[TAG] Clean lost of connection with the broker ", broker_address, ". Error Code = ", ec)
        print("[TAG] Exiting program")
    else:
        print("[TAG] Connection lost with the broker ", broker_address, ". Error Code = ", ec)
        # test what logging.info does exactly
        # logging.info("Connection lost with the broker ", broker_address, ". Error Code = ", ec)
        print("[TAG] Exiting program.")
    exit()


# call back method for messages
def on_message(client, userdata, message):
    client.message_received_flag = True
    if debug_print: print("[TAG] Message topic =", message.topic)
    if debug_print: print("[TAG] Message qos =", message.qos)
    # handling json
    data = get_id_and_coord(str(message.payload.decode("utf-8")))  # updates the variables of the class and return
    # a string with info for debug
    if debug_print: print(data)


# start
def start():
    # MAIN - connection
    global client  # to be able to write if necessary in those variables
    client.loop_start()  # start a loop like an update to check for call backs
    print("[TAG] Trying to connect to the broker ", broker_address)
    client.connect(broker_address)  # connect to the specified broker
    print("[TAG] Waiting for connection")

    # while not client.connected_flag or not client.failed_connection_flag:
    #    time.sleep(0.1)
    #    print("in the while")
    while not client.connected_flag:
        print("[TAG] ...")
        time.sleep(0.5)
    # The callbacks will continue to be triggered


# Variables
TOPIC = "None"
broker_address = None

# For test purpose
# When we launched the script itself (script not used as a class)
if __name__ == "__main__":
    global client, TOPIC, broker_address  # to be able to write if necessary in those variables

    # add flags in the class for debugging purposes
    mqtt.Client.connected_flag = False
    mqtt.Client.disconnected_flag = False
    mqtt.Client.failed_connection_flag = False
    mqtt.Client.message_received_flag = False

    # declarations
    TOPIC = "tags"
    broker_address = "172.30.4.44"  # define broker address
    client = mqtt.Client("rasp_GI")  # instanciate the client with its name

    # adding call back methods
    client.on_connect = on_connect  # bind call back method
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    # start communication
    start()
    while True:
        None  # wait for the packets to coooome from the broker
        # the call back on_message will trigger by it self as soon as a message is sent from the broker
        # on the topic we are listening


# Warning : This class is not related to a specific tag !!!
class Gateway:  # the broker in MQTT vocab
    # init
    def __init__(self, user_topic, user_broker_address, user_client_name):
        global client, TOPIC, broker_address  # to be able to write if necessary in those variables

        # add flags in the class for debugging purposes
        mqtt.Client.connected_flag = False
        mqtt.Client.disconnected_flag = False
        mqtt.Client.failed_connection_flag = False
        mqtt.Client.message_received_flag = False

        # declarations
        TOPIC = user_topic  # "tags"
        broker_address = user_broker_address  # "172.30.4.44"  # define broker address
        client = mqtt.Client(user_client_name)  # "rasp_GI")  # instanciate the client with its name

        # adding call back methods
        client.on_connect = on_connect  # bind call back method
        client.on_disconnect = on_disconnect
        client.on_message = on_message

        # start communication
        start()

    def get_coord(self, tag_id_requested):
        try:
            if debug_print: print("[TAG] tags_id[] = ", tags_id)
            id_tag_requested_index = tags_id.index(tag_id_requested)  # check if this id has been already received
            if debug_print: print("[TAG] Tag id requested = ",tag_id_requested, " found index = " ,id_tag_requested_index)
            return tags_coord[id_tag_requested_index]
        except ValueError:
            print("[TAG] THE REQUESTED ID TAG :", tag_id_requested, " IS NOT IN THE LIST !")
            return [0, 0, 0]

    # end process
    def close(self):
        exit()

'''
    # The following remark is linked to the wrong believe (that came in first place) that this class can be efficiently
    # be linked to a specific tag
    #
    # IMPORTANT remark : if there are more than on tag connected to the gateway, then the latter will broadcast coord
    # from all the tags connected to it on "tags" topic.
    # Therefore "on_message" will trigger for info about all the tags
    # BUT we only want to update coord if they are the ones of THIS tag (linked to this instance)
    # SO we need to know the id of the right tag to link it to its right instance
    def show_id(self):
        # connect only the WANTED tag to the gateway
        enable_debugging()  # the id of the tag will be displayed in console, note the id of this particular tag
        # then use set_id() with the right tag id

    def set_id(self, this_tag_id):
        self.id = this_tag_id  # at this time it is 26920

    def get_id(self):
        return self.id

    def get_coord(self):
        if id == self.id:
            self.coord = coord
        return self.coord
'''
