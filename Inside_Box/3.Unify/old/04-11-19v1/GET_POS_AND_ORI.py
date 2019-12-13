import Pozyx_MQTTvCONSOLE as pozyx

print("this is a test")
myTag = pozyx.Tag("tags", "172.30.4.44", "rasp_GI")
myTag.start()

try:
    # keep the program running
    while True:
        None

# use CTRL + \ to quit
except KeyboardInterrupt: # never triggered, don't know why
    print("Exception thrown. Exiting Program")
    exit()