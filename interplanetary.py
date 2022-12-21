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
        circle = b2FixtureDef(shape=b2CircleShape(radius=3), density=100, friction=0.5, restitution=0.5)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, 0), fixtures=circle)

        # Earth
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.7), density=50, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / r1)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, r1), fixtures=circle_small,
                                     linearVelocity=(V, 0))

        # Satellite
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.3), density=0.01, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / r1) + math.sqrt(self.G * self.world.bodies[2].mass / abs(r1-r2))
        #V = math.sqrt(2 * self.G * self.world.bodies[2].mass / abs(r1 - r2)) + math.sqrt(self.G * self.world.bodies[1].mass / r1)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(0, r2), fixtures=circle_small,
                                     linearVelocity=(V, 0))
        #Mars
        circle_small = b2FixtureDef(shape=b2CircleShape(radius=0.7), density=67, friction=0.5, restitution=0.2)
        V = math.sqrt(self.G * self.world.bodies[1].mass / r3)
        self.world.CreateDynamicBody(type=b2_dynamicBody, position=b2Vec2(-r3, 0), fixtures=circle_small,
                                     linearVelocity=(0, V))

    def Step(self, settings):
        super(Simulation, self).Step(settings)
        global r1, r2, T2, r3
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
        T_big = 2 * math.pi * math.sqrt(r1 ** 3 / (self.G * self.world.bodies[1].mass))
        time = self.stepCount / self.settings.hz
        if abs(time - T_big) < 0.0025:
            V = math.sqrt(2 * self.G * self.world.bodies[2].mass / abs(r1 - r2)) + math.sqrt(
                self.G * self.world.bodies[1].mass / r1)
            self.world.bodies[3].linearVelocity = (
                V * self.world.bodies[3].linearVelocity[0] / self.world.bodies[3].linearVelocity.length,
                V * self.world.bodies[3].linearVelocity[1] / self.world.bodies[3].linearVelocity.length
            )
        if time == 5:
            maneuver_start_t = time
            V = math.sqrt(self.G * self.world.bodies[1].mass / self.world.bodies[3].position.length)
            self.world.bodies[3].linearVelocity = (
                V * self.world.bodies[3].linearVelocity[0] / self.world.bodies[3].linearVelocity.length,
                V * self.world.bodies[3].linearVelocity[1] / self.world.bodies[3].linearVelocity.length
            )
        if time == 6:
            V0 = math.sqrt(self.G * self.world.bodies[1].mass / self.world.bodies[3].position.length)
            V = math.sqrt((V0 ** 2 * (2 * (r3-5) / (self.world.bodies[3].position.length + (r3-5)))))
            self.world.bodies[3].linearVelocity = (
                V * self.world.bodies[3].linearVelocity[0] / self.world.bodies[3].linearVelocity.length,
                V * self.world.bodies[3].linearVelocity[1] / self.world.bodies[3].linearVelocity.length
            )
            T2 = 2 * math.pi * math.sqrt(((self.world.bodies[3].position.length + r3 - 5) / 2) ** 3 / (self.G * self.world.bodies[1].mass))

        if time == 19.8:
            orbit = self.world.bodies[3].position-self.world.bodies[4].position
            V = math.sqrt(self.G * self.world.bodies[1].mass / r3) + math.sqrt(self.G * self.world.bodies[4].mass / abs(orbit.length))
            self.world.bodies[3].linearVelocity = (
                V * self.world.bodies[3].linearVelocity[0] / self.world.bodies[3].linearVelocity.length,
                V * self.world.bodies[3].linearVelocity[1] / self.world.bodies[3].linearVelocity.length
            )

r1 = 45
r2 = 50
r3 = 400
T2 = 9999
maneuver_start_t = 0
Simulation().run()
