# NeuroGoGo

This repository contains the program that implements an SSVEP-based EEG control system for controlling a wheelchair. 

## Setup

Instructions for setting up the program for use with an FT232H breakout board can be found [here](https://learn.adafruit.com/circuitpython-on-any-computer-with-ft232h).

## Usage

To run the program, use:
```
python3 Application/ngg.py
```

### Training

### data.pkl

The parameters set by the users during use is stored in a serialized savefile titled ```data.pkl```. 