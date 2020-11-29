import os
import time
import threading
try:
    os.environ["BLINKA_FT232H"] = "1"
    import board
except:
    print("Failure importing board")
    exit()
import digitalio

class GPIO_helper:

    def __init__(self, left_freq, right_freq, m3_freq, m4_freq):
        self.QUIT_FLAG = False # signal for Daemon threads to exit

        # Set up GPIO I/O for the four LEDs
        self.led_left = digitalio.DigitalInOut(board.C0)
        self.led_right = digitalio.DigitalInOut(board.C1)
        self.led_m3 = digitalio.DigitalInOut(board.C2)
        self.led_m4 = digitalio.DigitalInOut(board.C3)

        # Set up GPIO I/O for the movements

        self.mov_left = digitalio.DigitalInOut(board.C4)
        self.mov_right = digitalio.DigitalInOut(board.C5)
        self.mov_m3 = digitalio.DigitalInOut(board.C6)
        self.mov_m4 = digitalio.DigitalInOut(board.C7)

        for output in [self.led_left, self.led_right, self.led_m3, self.led_m4, self.mov_left, self.mov_right, self.mov_m3, self.mov_m4]:
            output.direction = digitalio.Direction.OUTPUT
            output.value = False

        # A dynamically updated dictionary for LED frequencies (in Hz)
        self.frequencies = {self.led_left : 1, self.led_right : 2, self.led_m3 : 3, self.led_m4 : 4}

        # Dictionary for addressing directions with strings
        self.directions = {"Left" : self.mov_left, "Right" : self.mov_right, "m3" : self.mov_m3, "m4" : self.mov_m4}

        # Dictionary for addressing LEDs with strings
        self.leds = {"Left" : self.led_left, "Right" : self.led_right, "m3" : self.led_m3, "m4" : self.led_m4}

        # Start flashing light threads for each LED
        self.threads = []
        for i in [self.led_left, self.led_right, self.led_m3, self.led_m4]:
            t = threading.Thread(target=self.flash_led, args=(i, self.frequencies))
            self.threads.append(t)
            t.setDaemon(True)
            t.start()

    def update_freq(self, led, new_freq):
        self.frequencies[self.leds[led]] = int(new_freq)

    # Thread turns LED off at frequency specified in 
    def flash_led(self, led, frequencies):
        while not self.QUIT_FLAG:
            led.value = True
            time.sleep((1 / self.frequencies[led]) / 2)
            led.value = False
            time.sleep((1 / self.frequencies[led]) / 2)

    # Output on
    def output_on(self, direction):
        self.directions[direction].value = True

    # Output off
    def output_off(self, direction):
        self.directions[direction].value = False

    # Return whether an output is on
    def output_val(self, direction):
        return self.directions[direction].value

    # Ends the flashing
    def quit(self):

        # Signals threads to end
        self.QUIT_FLAG = True
        for t in self.threads:
            t.join()

        # Turn off every LED
        for output in [self.led_left, self.led_right, self.led_m3, self.led_m4, self.mov_left, self.mov_right, self.mov_m3, self.mov_m4]:
            output.value = False


if __name__ == "__main__":
    helper = GPIO_helper(1, 2, 3, 4)

    command = input()
    while command != "next":
        new_freqs = command.split()
        helper.update_freq("Left", new_freqs[0])
        helper.update_freq("Right", new_freqs[1])
        helper.update_freq("m3", new_freqs[2])
        helper.update_freq("m4", new_freqs[3])
        command = input()

    command = input()
    while command != "exit":
        if (helper.output_val(command)):
            helper.output_off(command)
        else:
            helper.output_on(command)
        command = input()

    helper.quit()


