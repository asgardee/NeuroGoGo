import time
import board
import digitalio
import testing_and_making_data as test

led01 = digitalio.DigitalInOut(board.C0)
led02 = digitalio.DigitalInOut(board.C1)
led01.direction = digitalio.Direction.OUTPUT
led02.direction = digitalio.Direction.OUTPUT

while True:
    if test.ACTION == "left":
        led01.value = True

    elif test.ACTION == "right":
        led02.value = True
    else:
        led01.value = False
        led02.value = False
