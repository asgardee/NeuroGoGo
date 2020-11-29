import time
import board
import digitalio

left_command = digitalio.DigitalInOut(board.C0)
right_command = digitalio.DigitalInOut(board.C1)
LED_rightcommand = digitalio.DigitalInOut(board.C2)
LED_leftcommand= digitalio.DigitalInOut(board.C3)


left_command.direction = digitalio.Direction.OUTPUT
right_command.direction = digitalio.Direction.OUTPUT
LED_leftcommand.direction = digitalio.Direction.OUTPUT #Is the SSVEP LED output for left
LED_rightcommand.direction = digitalio.Direction.OUTPUT #is the SSVEP LED output for right

def wheelchair_command(action):
    #Takes in an action and outputs to the GPIO
    if action == "left":
        left_command.value = True

    elif action == "right":
        right_command.value = True
    else:
        left_command.value = False
        right_command.value = False

while True:
    LED_leftcommand.value = True
    time.sleep(0.05)  #(0.0415) for right command
    LED_leftcommand.value = False
    time.sleep(0.05) #(0.0415) for right command