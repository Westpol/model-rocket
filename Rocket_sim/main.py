import pygame
import time
import math

'''
ALL DISTANCE VALUES IN METERS, SET SCALE ACCORDINGLY
ALL ROTATIONAL CALUES IN RADIANS, SET SCALE ACCORDINGLY
'''


class Rocket:

    def __init__(self):
        self.lastUpdate = time.time()
        self.x1 = 0     # meters
        self.x2 = 0     # meters
        self.x3 = 10    # meters
        self.rx1 = 0    # meters
        self.rx2 = 0    # meters
        self.rx3 = 0    # meters
        self.thrust = 0     # percent from 0 to 1
        self.maxThrustForce = 1.5    # kilograms
        self.weight = 0.832    # kilogram

    def thrust_curve(self):
        return self.maxThrustForce * (1 / (1 + (math.e ** (- 8 * (self.thrust - 0.45)))))

    def update(self):

        self.lastUpdate = time.time()


class UI:

    def __init__(self, rocketClass):
        self.rocketClass = rocketClass

    def update(self):
        self.rocketClass.update()


class FC:
    def __init__(self):
        pass


if __name__ == '__main__':
    pygame.init()
    display = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

    rocket = Rocket()
    ui = UI(rocket)
    for i in range(100):
        ui.update()
        ui.rocketClass.thrust = i / 100
        print(str(i) + "%" + "   =   " + str(ui.rocketClass.thrust_curve()))
