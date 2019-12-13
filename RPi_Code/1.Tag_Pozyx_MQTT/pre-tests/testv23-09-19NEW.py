import time
import keyboard


close = False


# def call back
def on_press(key):
    print("Key press detected")


def on_release(key):
    print("Key release detected")
    if key == keyboard.Key.esc:
        close = True


print('This is the test')


keyboard.on_press(on_release)


while True:
    print("still in loop")
    if close:
        print('you asked to close the program')
        exit()
