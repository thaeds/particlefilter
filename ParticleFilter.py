#!/bin/python3

from enum import Enum
import math
import random
import copy
import sys


class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

def parseaction():
    char = getch()
    if char == 'w':
        action = Action.MOVE_FORWARDS
    elif char == 'a':
        action = Action.TURN_LEFT
    elif char == 's':
        action = Action.MOVE_BACKWARDS
    elif char == 'd':
        action = Action.TURN_RIGHT
    else:
        action = Action.NONE
        sys.exit()
    return action


def main():
    #knownmap = Map([['-', '-', '-', '-', '-', '-', '-', '-', '-',  '-'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|',  '',  '',  '',  '',  '',  '',  '',  '',  '|'],
    #           ['|', '_', '_', '_', '_', '_', '_', '_', '_',  '|']])

    originalmap = ["----------",
                    "|.       |",
                    "|        |",
                    "|   .    |",
                    "|        |",
                    "|        |",
                    "| .      |",
                    "|     .  |",
                    "|        |",
                    "----------"]

    knownmap = Map(copy.deepcopy(originalmap))
    pose = Pose(5, 5, 0)
    noise = Noise(lambda x : random.uniform(x-.1, x+.1))
    exploration = Noise(lambda x : random.gauss(x, .5))
    noisyodometry = NoisyOdometry(lambda pose, action, knownmap: knownmap.move(pose, action).pathfrom(pose), noise)
    def sensorfunction(pose, knownmap):
        distance = 0
        while True:
            x = int(pose.x + (math.cos(pose.t) * distance))
            y = int(pose.y + (math.sin(pose.t) * distance))
            if x < 0 or x >= knownmap.cols or y < 0 or y >= knownmap.rows or knownmap[y][x]:
                return distance
            else:
                distance += .1

    noisysensor = NoisySensor(sensorfunction, noise)
    particles = Particles(knownmap, 500, exploration, Sensor(sensorfunction))


    while True:
        print('\n'.join(originalmap))
        reading = noisysensor.read(pose, knownmap)
        particles.updatesensor(reading)
        print("Range sensor detected an obstacle: " + str(reading) + " spaces ahead")
        particles.reweight()
        particles.resample()
        action = parseaction()
        movement = noisyodometry.read(pose, action, knownmap)
        pose = knownmap.move(pose, action)
        particles.updatemotion(movement)
        print("Robot moved to: " + str(pose.x) + ", " + str(pose.y) + ", " + str(pose.t))



# all non empty grid spaces are obstacles
class Map(list):
    def __init__(self, map):
        list.__init__(self, [[0 if item == " " else 1 for item in row] for row in map[:]])
        self.rows = len(map)
        self.cols = len(map[0])

    def boundcheck(self, pose):
        return pose.x >= 0 and pose.x < self.cols and pose.y >= 0 and pose.y < self.rows

    def isvalidaction(self, pose, action):
        if self[pose.y][pose.x]:
            return False
        elif action == Action.MOVE_FORWARDS:
            newpose = Pose(int(pose.x + math.cos(pose.t)), int(pose.y + math.sin(pose.t)), pose.t)
            return self.boundcheck(newpose) and not self[newpose.y][newpose.x]
        elif action == Action.MOVE_BACKWARDS:
            newpose = Pose(int(pose.x - math.cos(pose.t)), int(pose.y - math.sin(pose.t)), pose.t)
            return self.boundcheck(newpose) and not self[newpose.y][newpose.x]
        else:
            return True

    def move(self, pose, action):
        if not self.isvalidaction(pose, action):
            return pose
        elif action == Action.TURN_LEFT:
            return Pose(pose.x, pose.y, pose.t - math.pi/2)
        elif action == Action.TURN_RIGHT:
            return Pose(pose.x, pose.y, pose.t + math.pi/2)
        elif action == Action.MOVE_FORWARDS:
            return Pose(int(pose.x + math.cos(pose.t)), int(pose.y + math.sin(pose.t)), pose.t)
        elif action == Action.MOVE_BACKWARDS:
            return Pose(int(pose.x - math.cos(pose.t)), int(pose.y - math.sin(pose.t)), pose.t)
        else:
            return Pose(pose.x, pose.y, pose.t)

    def validpositions(self):
        count = 0
        for row in self:
            for pos in row:
                if not pos:
                    count += 1
        return count

    def __str__(self):
        outstr = str()
        outstr += "\n\n"
        outstr = "-----------> X\n|\n|\n|\n|\n|\nv\n\nY\n\n"
        for row in self:
            outstr += " ".join(['{:3}'.format(num) for num in row]) + '\n'
        outstr += "\n\n\n\n\n"
        return outstr


class Action(Enum):
    NONE = 0
    MOVE_FORWARDS = 1
    MOVE_BACKWARDS = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4


class Noise:
    def __init__(self, noise):
        self.noise = noise

    def read(self, value):
        return self.noise(value)


class Odometry:
    def __init__(self, odometry):
        self.odometry = odometry

    def read(self, pose, action, map):
        return self.odometry(pose, action, map)


class NoisyOdometry:
    def __init__(self, odometry, noise):
        Odometry.__init__(self, odometry)
        self.noise = noise

    def read(self, pose, action, map):
        deltaPose = Odometry.read(self, pose, action, map)
        noisyx = self.noise.read(deltaPose.x)
        noisyy = self.noise.read(deltaPose.y)
        noisyt = self.noise.read(deltaPose.t)
        return Pose(noisyx, noisyy, noisyt)


class Sensor:
    def __init__(self, sensor):
        self.sensor = sensor

    def read(self, pose, map):
        ret =  self.sensor(pose, map)
        return ret


class NoisySensor(Sensor):
    def __init__(self, sensorfn, noise):
        Sensor.__init__(self, sensorfn)
        self.noise = noise

    def read(self, pose, map):
        return self.noise.read(Sensor.read(self, pose, map))


class Pose:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t

    def pathfrom(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dist = math.pow(math.pow(dx, 2) + math.pow(dy, 2), .5)
        rpdt = math.atan2(dy, dx) - other.t
        rdx = dist * math.cos(rpdt)
        rdy = dist * math.sin(rpdt)
        dt = self.t - other.t
        #the below are in robot frame co-ordinates
        return Pose(rdx, rdy, dt)


class Particles:
    def __init__(self, map, count, noise, sensor):
        self.originalmap = map
        self.map = Map([[0 for i in range(map.cols)] for j in range(map.rows)])
        self.count = count
        self.noise = noise
        self.particles = []
        self.sensor = sensor
        tstep = math.pi / 2
        for x in range(0, map.cols):
            for y in range(0, map.rows):
                for t in range(0, 4):
                    p = Pose(x, y, t * tstep)
                    self.particles.append(Particle(p, 1.0/self.count))
       #while len(self.particles) < self.count:
       #     p = Pose(random.randint(0, self.map.cols), random.randint(0, self.map.rows), 100 * random.random() % math.pi)
       #     self.particles.append(Particle(p, 1/self.count))

    def resample(self):
        newparticles = []
        while len(newparticles) < self.count:
            particle = self.particles[random.randint(0, len(self.particles) - 1)]
            if random.random() < particle.weight:
                x = self.noise.read(particle.pose.x)
                y = self.noise.read(particle.pose.y)
                t = self.noise.read(particle.pose.t)
                newparticle = Particle(Pose(x, y, t), 1)
                if self.map.boundcheck(newparticle.pose):
                    newparticles.append(newparticle)
        self.particles = newparticles
        for r in range(self.map.rows):
            for c in range(self.map.cols):
                self.map[r][c] = 0
        for p in self.particles:
            c = int(p.pose.x)
            r = int(p.pose.y)
            self.map[r][c] += 1
        print(self.map)

    def reweight(self):
        bias = 0.1 #10% of weight split evenly among particles
        totalweight = sum([x.weight for x in self.particles])
        totalweight *= bias
        for particle in self.particles:
            particle.weight = particle.weight / ((1 + bias) * totalweight) + totalweight * bias / self.count

    def updatemotion(self, deltapose):
        for particle in self.particles:
            particle.updatemotion(deltapose)

    def updatesensor(self, reading):
        for particle in self.particles:
            particle.updatesensor(self.sensor, reading, self.originalmap)


class Particle:
    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def updatemotion(self, deltapose):
        self.pose.x += deltapose.x * math.cos(self.pose.t) - deltapose.y * math.sin(self.pose.t)
        self.pose.y += deltapose.y * math.cos(self.pose.t) + deltapose.x * math.sin(self.pose.t)
        self.pose.t += deltapose.t

    def updatesensor(self, sensor, reading, map):
        expectedReading = sensor.read(self.pose, map)
        covar =.2
        self.weight = max(1 - (reading - expectedReading) / .2, 0)


if __name__ == '__main__':
    main()

