from Box2D import *
from Box2D.examples.framework import Framework
import math


class Simulation(Framework):
    def __init__(self):
        super(Simulation, self).__init__()
        self.settings.hz = 200
        self.velocityIterations = 1
        global r1, r2

        # Default gravity disable
        self.world.gravity = (0.0, 0.0)
        # Gravity constant
        self.G = 100

        # Sun
        circle = b2FixtureDef(shape=b2CircleShape(radius=3), density=80, friction=0.5, restitution=0.5)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, 0), fixtures=circle)

        # Earth
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.7), density=50, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / r1)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, r1), fixtures=circle_small,
                                     linearVelocity=(V, 0))

        # Satellite
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.3), density=0.01, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / r1) + math.sqrt(self.G * self.world.bodies[2].mass / abs(r1 - r2))
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, r2), fixtures=circle_small,
                                     linearVelocity=(V, 0))

    def Step(self, settings):
        super(Simulation, self).Step(settings)
        global r1, r2, T2
        global maneuver_start_t
        # Simulate the Newton's gravity
        for bi in self.world.bodies[2:]:
            for bk in self.world.bodies:
                if bi == bk:
                    continue

                pi, pk = bi.worldCenter, bk.worldCenter
                mi, mk = bi.mass, bk.mass
                delta = pk - pi
                r = delta.length
                if abs(r) < 1.0:
                    r = 1.0

                force = self.G * mi * mk / (r * r)
                delta.Normalize()
                bi.ApplyForce(force * delta, pi, True)

        #   print(self.world.bodies[2].position, self.stepCount / self.settings.hz, self.world.bodies[2].linearVelocity)
        T = 2 * math.pi * math.sqrt(abs(r1 - r2) ** 3 / (self.G * self.world.bodies[1].mass))
        time = self.stepCount / self.settings.hz
        print(T)
        '''if abs(time - T) < 0.0025:
            print(T)
            print(time, self.world.bodies[2].position)'''

        '''if abs(time - 2 * T) < 0.0025:
            print("START")
            maneuver_start_t = time
            V = math.sqrt((self.world.bodies[2].linearVelocity.length ** 2 * (2 * r2 / (r1 + r2))))
            self.world.bodies[2].linearVelocity = (
                V * self.world.bodies[2].linearVelocity[0] / self.world.bodies[2].linearVelocity.length,
                V * self.world.bodies[2].linearVelocity[1] / self.world.bodies[2].linearVelocity.length
            )
            T2 = 2 * math.pi * math.sqrt(((r1 + r2) / 2) ** 3 / (self.G * self.world.bodies[1].mass))
            print(T2)
        #   print(time - maneuver_start_t - T2 / 2)
        if abs(time - maneuver_start_t - T2 / 2) < 0.0025:
            V = math.sqrt(self.G * self.world.bodies[1].mass / r2)
            #   self.world.bodies[2].linearVelocity = (-V, 0)
            self.world.bodies[2].linearVelocity = (
                V * self.world.bodies[2].linearVelocity[0] / self.world.bodies[2].linearVelocity.length,
                V * self.world.bodies[2].linearVelocity[1] / self.world.bodies[2].linearVelocity.length
            )
            print("END")'''

        #   print(self.world.bodies[2].position.length)





r1 = 45
r2 = 40
T2 = 9999
maneuver_start_t = 0
Simulation().run()
