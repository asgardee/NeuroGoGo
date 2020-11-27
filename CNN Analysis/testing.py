from pylsl import StreamInlet, resolve_stream
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import style
from collections import deque
import cv2
import os
import random
import tensorflow as tf
import command


def file_input(file):
    #Purpose: to import the file to be analyzed


def predicted_command(file):
    #Purpose: to take in a file and predict the command
    #Input: completed BCI measurement file
    #Output: predicted command

command.wheelchair_command(predicted_command(file_input(file))) #Takes the file, imports it,
# predicts the command it should have, then creates an output to the GPIO

