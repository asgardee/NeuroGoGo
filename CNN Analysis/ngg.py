# https://pythonprogramming.net/object-oriented-programming-crash-course-tkinter/

import tkinter as tk
import pickle
import os
import GPIO_helper

# Contains the tkinter and logic for the entire program
class Application(tk.Tk):

    def __init__(self, *args, **kwargs):

        tk.Tk.__init__(self, *args, **kwargs)

        # Dictionary contains all params needed for NN, etc.
        # This will contain all the information in the savefile
        self.params = {}
        
        # Check for savefile and load if exists
        if os.path.exists("data.pkl"):
            savefile = open("data.pkl", "rb")
            self.params = pickle.load(savefile)
            savefile.close()
        else: # default values
            self.params["Left"] = 8
            self.params["Right"] = 12
            self.params["Forward"] = 15
            self.params["Reverse"] = 18

        # Helper to interface with GPIO
        self.helper = GPIO_helper.GPIO_helper(self.params["Left"], self.params["Right"], self.params["Forward"], self.params["Reverse"])

        # Bind closing sequence
        self.protocol("WM_DELETE_WINDOW", self.closing_seq)

        # Container frame contains all subsequent frames
        # TODO... is this necessary?
        container = tk.Frame(self)
        container.pack()

        # Create frames and store them into dictionary
        self.frames = {}
        frame = MainPage(container, self)
        self.frames[MainPage] = frame
        frame.grid(row=0, column=0, sticky="nsew")
        frame = TrainPage(container, self)
        self.frames[TrainPage] = frame
        frame.grid(row=0, column=0, sticky="nsew")

        # Begin with main page
        self.show_frame(MainPage)

        # start update process
        self.after(1000, self.update)


    # Change displayed frame
    def show_frame(self, new_frame):

        frame = self.frames[new_frame]
        frame.tkraise()


    # Actions to be taken prior to closing
    def closing_seq(self):
        
        # Saves current params
        savefile = open("data.pkl", "wb")
        pickle.dump(self.params, savefile)
        savefile.close()

        # Closes helper
        self.helper.quit()

        # Closes window
        self.destroy()


    # Returns a given parameter stored in self.params
    def get_param(self, query):

        return self.params[query];


    # Returns a given parameter in self.params
    def set_param(self, query, new_val):
        if query in self.params:
            self.params[query] = new_val


    # This function will be the part where you check for new inputs
    def update(self):
        #print("I'm updating!")
        self.after(1000, self.update)


    def output_on(self, event, direction):
        if direction == "Left":
            self.helper.output_on("Left")
        elif direction == "Right":
            self.helper.output_on("Right")
        elif direction == "Forward":
            self.helper.output_on("m3")
        elif direction == "Reverse":
            self.helper.output_on("m4")

    def output_off(self, event, direction):
        if direction == "Left":
            self.helper.output_off("Left")
        elif direction == "Right":
            self.helper.output_off("Right")
        elif direction == "Forward":
            self.helper.output_off("m3")
        elif direction == "Reverse":
            self.helper.output_off("m4")

    def update_freq(self, direction, new_freq):
        if direction == "Left":
            self.helper.update_freq("Left", new_freq)
        elif direction == "Right":
            self.helper.update_freq("Right", new_freq)
        elif direction == "Forward":
            self.helper.update_freq("m3", new_freq)
        elif direction == "Reverse":
            self.helper.update_freq("m4", new_freq)


# Frame layout for controlling the user frequencies
# Warning: code below is garbage but I don't know how to make it cleaner
class MainPage(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self, parent, bg="white")

        # Maintain reference to parent app
        self.controller = controller

        # Maintain pointers to the following elements with a dictionary
        self.frames = {}
        self.labels = {}
        self.entries = {}
        self.btn_update = {}
        self.btn_test = {}

        column = 0

        for direction in ["Left", "Right", "Forward", "Reverse"]:
            print("Creating " + direction)
            self.frames[direction] = tk.Frame(self, bg="white")
            self.labels[direction] = tk.Label(self.frames[direction], text=direction + ": " + str(self.controller.get_param(direction)) + " Hz", bg="white")
            self.entries[direction] = tk.Entry(self.frames[direction])
            self.entries[direction].bind("<Return>", lambda event, arg_direction=direction: self.update_frequency(event, arg_direction))
            self.btn_update[direction] = tk.Button(self.frames[direction], text="Update " + direction)
            self.btn_update[direction].bind("<ButtonPress>", lambda event, arg_direction=direction: self.update_frequency(event, arg_direction))
            self.btn_test[direction] = tk.Button(self.frames[direction], text = "Test")
            self.btn_test[direction].bind("<ButtonPress>", lambda event, arg_direction=direction: self.controller.output_on(event, arg_direction))
            self.btn_test[direction].bind("<ButtonRelease>", lambda event, arg_direction=direction: self.controller.output_off(event, arg_direction))
            self.labels[direction].pack()
            self.entries[direction].pack()
            self.btn_update[direction].pack()
            self.btn_test[direction].pack()
            self.frames[direction].grid(row=0, column=column, padx=5, pady=5)
            column += 1


        # Button to go to training page
        self.frm_train = tk.Frame(self)
        self.btn_train = tk.Button(self.frm_train, text="Train", command=lambda:controller.show_frame(TrainPage))
        self.btn_train.pack()
        self.frm_train.grid(row=0, column=column, padx=5, pady=5)


    def update_frequency(self, event, direction):
        new_value = self.entries[direction].get()
        if new_value.isnumeric() and int(new_value) >= 1:
            self.labels[direction]["text"] = direction + ": " + str(new_value) + " Hz"
            self.controller.set_param(direction, int(new_value))
            self.controller.update_freq(direction, int(new_value))

    # Update frequency for moving left TESTING
    def btn_left_action(self, event):
        new_value = self.ent_left.get()
        if new_value.isnumeric():
            self.lbl_left["text"] = "Left: " + str(new_value) + " Hz"
            self.controller.set_param("left", int(new_value))
        else:
            print("Value is not valid")


    # Update frequency for moving right TESTING
    def btn_right_action(self, test):
        new_value = self.ent_right.get()
        if new_value.isnumeric():
            self.lbl_right["text"] = "Right: " + str(new_value) + " Hz"
            self.controller.set_param("right", int(new_value))


# Frame layout for the training function
class TrainPage(tk.Frame):

    def __init__(self, parent, controller):

        tk.Frame.__init__(self, parent)
        self.lbl_test = tk.Label(self, text="The functionality for training will go here")
        # Goes back to main page
        self.btn_back = tk.Button(self, text="Back", command=lambda: controller.show_frame(MainPage))

        self.lbl_test.pack();
        self.btn_back.pack();


if __name__ == "__main__":
    app = Application()
    app.mainloop()