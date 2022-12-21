from Box2D import *
from Box2D.examples.framework import Framework
import math


class Simulation(Framework):
    def __init__(self):
        super(Simulation, self).__init__()
        self.settings.hz = 200
        self.velocityIterations = 1
        global r1

        # Default gravity disable
        self.world.gravity = (0.0, 0.0)
        # Gravity constant
        self.G = 100

        # Planet
        circle = b2FixtureDef(shape=b2CircleShape(radius=10), density=4, friction=0.5, restitution=0.5)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, 0), fixtures=circle)

        # Satellite
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.7), density=1, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / r1)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, 40), fixtures=circle_small,
                                     linearVelocity=(V, 0))

    def Step(self, settings):
        super(Simulation, self).Step(settings)
        global r1, r2, T2
        global maneuver_start_t
        for bi in self.world.bodies[2:3]:
            for bk in self.world.bodies[1:2]:
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

        T = 2 * math.pi * math.sqrt(r1 ** 3 / (self.G * self.world.bodies[1].mass))
        time = self.stepCount / self.settings.hz
        if abs(time - 2 * T) < 0.0025:
            print("START")
            maneuver_start_t = time
            V = math.sqrt((self.world.bodies[2].linearVelocity.length ** 2 * (2 * r2 / (r1 + r2))))
            self.world.bodies[2].linearVelocity = (
                V * self.world.bodies[2].linearVelocity[0] / self.world.bodies[2].linearVelocity.length,
                V * self.world.bodies[2].linearVelocity[1] / self.world.bodies[2].linearVelocity.length
            )
            T2 = 2 * math.pi * math.sqrt(((r1 + r2) / 2) ** 3 / (self.G * self.world.bodies[1].mass))
        if abs(time - maneuver_start_t - T2 / 2) < 0.0025:
            V = math.sqrt(self.G * self.world.bodies[1].mass / r2)
            self.world.bodies[2].linearVelocity = (
                V * self.world.bodies[2].linearVelocity[0] / self.world.bodies[2].linearVelocity.length,
                V * self.world.bodies[2].linearVelocity[1] / self.world.bodies[2].linearVelocity.length
            )

r1 = 40
r2 = 20
T2 = 9999
maneuver_start_t = 0
Simulation().run()