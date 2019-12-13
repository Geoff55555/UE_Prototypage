import keyboard
# pay great attention to install keyboard with
# sudo pip3 install keyboard
# DON'T FORGER THE SUDO, IT WON'T WORK OTHERWISE

# add callback method on key press event
keyboard.on_press_key("g", lambda _:print("You pressed g"))
# as the on_press_key returns a key event to the lambda function,
# we need an argument for it. We named it _ because it'll be thrown away
# answer from black thunder : https://stackoverflow.com/questions/24072790/detect-key-press-in-python
# but creating a function that takes one argument and replacing lambda with this function doesn't work...
# mystery ???
try:
    while 1:
        None
except KeyboardInterrupt:
    print("KeyboardInterrupt exception launched")
    print("Closing Program")
    keyboard.unhook_all()
    exit()