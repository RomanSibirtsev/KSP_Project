from Box2D import *
from Box2D.examples.framework import Framework
import math


class Simulation(Framework):
    def __init__(self):
        super(Simulation, self).__init__()
        global a1

        # Default gravity disable
        self.world.gravity = (0.0, 0.0)
        # Gravity constant
        self.G = 100

        # Planet
        circle = b2FixtureDef(shape=b2CircleShape(radius=10), density = 1, friction=0.5, restitution=0.5)
        self.world.CreateBody(type=b2_dynamicBody, position=b2Vec2(0,0), fixtures=circle)

        # Satellite
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.2), density=1, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / a1)
        self.world.CreateBody(type=b2_dynamicBody, position=b2Vec2(0, 20), fixtures=circle_small, linearVelocity=(V, 0))

    def Step(self, settings):
        super(Simulation, self).Step(settings)
        global param
        global a1, a2

        # Simulate the Newton's gravity
        for bi in self.world.bodies:
            for bk in self.world.bodies:
                if bi == bk:
                    continue

                pi, pk = bi.worldCenter, bk.worldCenter
                mi, mk = bi.mass, bk.mass
                delta = pk - pi
                r = delta.length
                #print(delta, r)
                if abs(r) < 1.0:
                    r = 1.0

                force = self.G * mi * mk / (r * r)
                delta.Normalize()
                bi.ApplyForce(force * delta, pi, True)
            # print(self.world.bodies[2].linearVelocity.length)

        #print(self.world.bodies[2].position.length)
        #print(dir(self.world))
        if abs(self.world.bodies[2].position[0]) < 0.12 and param == 0:
            print("1\n")
            self.world.bodies[2].linearVelocity = (-math.sqrt((self.world.bodies[2].linearVelocity.length**2 * (2 * a2 / (a1 + a2)))), 0)
            param = 1

        if (abs(self.world.bodies[2].position[0]) < 0.12) and (param == 1) and (abs(self.world.bodies[2].position.length - a2) < 0.1):
            print("2\n")
            self.world.bodies[2].linearVelocity = (math.sqrt(self.G * self.world.bodies[1].mass / a2), 0)
            param = 2


a1 = 20
a2 = 40
param = 0
Simulation().run()
