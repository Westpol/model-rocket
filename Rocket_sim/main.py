import pygame
import serial


if __name__ == '__main__':
    arduino = serial.Serial()
    arduino.baudrate(115200)
    arduino.port = 'tst'
