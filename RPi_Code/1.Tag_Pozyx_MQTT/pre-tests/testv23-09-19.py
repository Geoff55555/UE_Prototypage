import time
from pynput import keyboard
from pynput import mouse

close = False


# def call back
def on_press(key):
    print("Key press detected")


def on_release(key):
    print("Key release detected")
    if key == keyboard.Key.esc:
        close = True


print('This is the test')

# script arrêté au with ! il ne fait qu'écouter et ne passe pas à la suite...
with keyboard.Listener(
    on_press=on_press,
    on_release=on_release) as listener:
    listener.join()

# non blocking fashion --> doesn't work - source : pypi.org
# listener = mouse.Listener(
#    on_press=on_press,
#    on_release=on_release)

# listener.start()

while True:
    print("still in loop")
    if close:
        print('you asked to close the program')
        exit()
