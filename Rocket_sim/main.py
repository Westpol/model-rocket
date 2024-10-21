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
        self.v_x1 = 0     # meters / second
        self.v_x2 = 0     # meters / second
        self.v_x3 = 0    # meters / second
        self.a_x1 = 0     # meters / second ** 2
        self.a_x2 = 0     # meters / second ** 2
        self.a_x3 = 0    # meters / second ** 2
        self.rx1 = 0    # meters
        self.rx2 = 0    # meters
        self.rx3 = 0    # meters
        self.thrust1 = 0     # percent from 0 to 1 of upper motor
        self.thrust2 = 0     # percent from 0 to 1 of lower motor
        self.maxThrustForce1 = 1    # kilograms
        self.maxThrustForce2 = 0.7    # kilograms
        self.angularMomentum = 0    # momentum around the z axis
        self.angle = 0      # z axis position
        self.weight = 0.832    # weight of craft in kilograms
        self.g = -9.81   # gravitational constant

    def thrust_curve(self):
        return 1 / (1 + (math.e ** (- 8 * (self.thrust1 - 0.45)))), 1 / (1 + (math.e ** (- 8 * (self.thrust2 - 0.45))))

    def update(self):
        deltaT = (time.time() - self.lastUpdate)
        thrust = self.thrust_curve()
        self.a_x3 = self.g * (self.weight - thrust[0] * self.maxThrustForce1 + thrust[1] * self.maxThrustForce2) * deltaT
        self.v_x3 += self.a_x3 * deltaT
        self.x3 += self.v_x3 * deltaT
        self.lastUpdate = time.time()


class UI:

    def __init__(self, rocketClass):
        self.rocketClass = rocketClass

    def update(self):
        self.rocketClass.update()


class FC:
    def __init__(self, p_t, d_t, ):
        pass


if __name__ == '__main__':
    pygame.init()
    display = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

    rocket = Rocket()
    ui = UI(rocket)
    for i in range(101):
        ui.rocketClass.thrust1 = i / 100
        ui.rocketClass.thrust2 = i / 100
        print(str(i) + "%" + "   =   " + str(ui.rocketClass.a_x3))
        ui.update()
        time.sleep(0.01)
