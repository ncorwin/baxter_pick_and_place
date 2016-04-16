#!/usr/bin/python
# Plot data from the PIC32 in python
# requries pyserial, matplotlib, and numpy
import serial
import time
import matplotlib.pyplot as plt
import numpy as np

port = '/dev/ttyUSB0' # the name of the serial port
ser = serial.Serial(port,230400,rtscts=1)


def Test(valve = 3):

  if valve == 0:
    ser.write("1 0/n")
  elif valve == 1:
    ser.write("0 1\n")
  elif valve == 2:
    ser.write("1 1\n")
  else:
    ser.write("0 0\n")

  return

def Grip(on = 0):

  if on == 1:
    ser.write("1 0\n")
  else:
    ser.write("1 1\n")

  return

def Reset(timer = 0.05):

  ser.write("0 1\n")
  time.sleep(timer)
  ser.write("1 1\n")
  return

def Throw():
  
  Grip()
  Reset(0.1)
  
  return

def main():

  return

if __name__ == '__main__':
  main()
