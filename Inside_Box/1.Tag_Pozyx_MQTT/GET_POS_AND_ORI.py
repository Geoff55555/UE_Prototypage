import Pozyx_MQTT as poz

print("this is a test")
myTag = poz("tags", "172.30.4.44", "rasp_GI")
myTag.start()

try:
    # keep the program running
    while True:
        None

# use CTRL + \ to quit
except KeyboardInterrupt: # never triggered, don't know why
    print("Exception thrown. Exiting Program")
    exit()