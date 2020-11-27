import time
import board
import digitalio

left_command = digitalio.DigitalInOut(board.C0)
right_command = digitalio.DigitalInOut(board.C1)
left_command.direction = digitalio.Direction.OUTPUT
right_command.direction = digitalio.Direction.OUTPUT

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
    wheelchair_command("left")
    time.sleep(0.5)
    wheelchair_command("none")
    time.sleep(0.5)
    wheelchair_command("right")
    time.sleep(0.5)
    wheelchair_command("none")
    time.sleep(0.5)