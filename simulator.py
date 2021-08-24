import pygame, sys, numpy
from pygame.locals import *

pygame.init()
FPS = 60
fpsClock = pygame.time.Clock()

DISPLAYSURF = pygame.display.set_mode((1000,1000),0,32)
pygame.display.set_caption("2D flight controller")

BLACK = (0,0,0)
WHITE = (255,255,255)

DISPLAYSURF.fill(WHITE)

def rotation(vector_in, angle_in):
    c,s = numpy.cos(angle_in), numpy.sin(angle_in)
    R = numpy.array(((c,-s),(s,c)))
    return R.dot(vector_in)


def draw_ship(position_in, angle_in, angle_gimbal_in): 
    body = numpy.array([0,50])
    middle = numpy.array([500,500])
    top = position_in - rotation(body, angle_in) + middle
    bottom = position_in + rotation(body, angle_in) + middle
    gimbal_bottom = bottom + rotation(body, angle_in + angle_gimbal_in)

    pygame.draw.line(DISPLAYSURF, BLACK, tuple(map(int,top)),tuple(map(int,bottom)),6)     
    pygame.draw.line(DISPLAYSURF, BLACK, tuple(map(int,bottom)),tuple(map(int,gimbal_bottom)),6) 

class MYSHIP:
    # test all features
    def __init__(self):
        self.weight = 0.1 # normalized
        self.length = 0.1 # normalized

        self.position = numpy.array([0,0],dtype=float)
        self.velocity = numpy.array([0,0],dtype=float)

        self.angle = 0 # rad
        self.rotation_speed = 0 # rad/s

        self.thrust = 0 # between 0 and 1
        self.throttle = 0 
        self.gimbal_angle = 0 # rad
        self.gimbal_speed = 0

    def acceleration(self): 
        F = numpy.array([0,1])*self.thrust
        gravity = numpy.array([0,-9.81],dtype=float)
        # gravity = numpy.array([0,0],dtype=float)
        accel = self.weight*rotation(F, self.angle + self.gimbal_angle)/self.weight
        return (accel + gravity)*9

    def torque(self):
        # simpler since 2D, cross product does not exist
        return numpy.sin(-self.gimbal_angle)/self.weight*self.length*self.thrust

    def time_step(self, step_size):
        self.position += self.velocity*step_size
        self.velocity += self.acceleration()*step_size

        self.angle += self.rotation_speed*step_size
        self.rotation_speed += self.torque()*step_size

        self.box_adjust()

        self.gimbal_angle += self.gimbal_speed*step_size
        self.thrust += self.throttle*step_size

    def box_adjust(self): 
        if self.position[0] > 450:
            self.position[0] -= 900
        elif self.position[0] < -450:
            self.position[0] += 900

        if self.position[1] > 450:
            self.position[1] -= 900
        elif self.position[1] < -450:
            self.position[1] += 900

        if self.angle > numpy.pi:
            self.angle -= 2*numpy.pi
        elif self.angle < numpy.pi:
            self.angle += 2*numpy.pi

        if self.gimbal_angle > numpy.pi/2:
            self.gimbal_angle = numpy.pi/2
        if self.gimbal_angle < -numpy.pi/2:
            self.gimbal_angle = -numpy.pi/2

        if self.thrust > 1000:
            self.thrust = 1000
        
    def flight_stabalizer(self):
        # return velocity, acceleration, angle, rotation, all to zero. 
        theta = self.angle
        omega = self.rotation_speed
        accel = 0 
        gimbal_target = 0 
        
        if self.rotation_speed > 0.01:
            gimbal_target = -numpy.pi/2
        elif self.rotation_speed < -0.01:
            gimbal_target = numpy.pi/2

        self.gimbal_speed += gimbal_target - self.gimbal_angle 

ship = MYSHIP()

while True: # Main game loop

    DISPLAYSURF.fill(WHITE)

    
    ship.time_step(1/FPS)
    draw_ship(-ship.position, ship.angle, ship.gimbal_angle)

    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYUP or event.type == pygame.KEYDOWN:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT: 
                    ship.gimbal_speed += 1
                if event.key == pygame.K_RIGHT:
                    ship.gimbal_speed -= 1

                if event.key == pygame.K_UP:
                    ship.throttle += 10
                if event.key == pygame.K_DOWN:
                    ship.throttle -= 10

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT: 
                    ship.gimbal_speed -= 1
                if event.key == pygame.K_RIGHT:
                    ship.gimbal_speed += 1

                if event.key == pygame.K_UP:
                    ship.throttle -= 10
                if event.key == pygame.K_DOWN:
                    ship.throttle += 10
        else:
            ship.flight_stabalizer()


    pygame.display.update()

    fpsClock.tick(FPS)



    