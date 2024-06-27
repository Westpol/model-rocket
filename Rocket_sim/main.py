import pygame
import serial


if __name__ == '__main__':
    arduino = serial.Serial("/dev/ttyACM0", baudrate=115200)

    while 1:
        print(arduino.readline())
