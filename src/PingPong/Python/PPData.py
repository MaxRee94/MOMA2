from typing import NewType, List, Tuple
from dataclasses import dataclass
from enum import IntEnum

from PPGeometry import *

# Unit type synonyms for readability.
Radian                = NewType("Radian", float)
Second                = NewType("Second", float)
Meter                 = NewType("Meter", float)
RadianPerSecond       = NewType("RadianPerSecond", float)
RadianPerSquareSecond = NewType("RadianPerSquareSecond", float)
MeterPerSecond        = NewType("MeterPerSecond", float)
MeterPerSquareSecond  = NewType("MeterPerSquareSecond", float)

def parse_time(data) -> Second:
    return Second(data)

# Dataclasses for joints, links, and the arm.
@dataclass
class Joint:
    jcol: str #Colour in hex format used for drawing the joint.
    jang: Radian #Current angle of the joint.
    jvel: RadianPerSecond = 0 #Angular velocity of the joint.

    def update(self, angle: Radian, velocity: RadianPerSecond):
        self.jang = angle
        self.jvel = velocity

    def from_json(data):
        color = data["colour"]
        angle = data["angle"]
        velocity = data["velocity"]

        return Joint(color, angle, velocity)

@dataclass
class Link:
    lcol: str #Colour in hex format used for drawing the link.
    llen: Meter #Length of the link.

    def update(self, length: Meter):
        self.llen = length

    def from_json(data):
        color = data["colour"]
        length = data["length"]

        return Link(color, length)
	
@dataclass
class Bat(Link):
    llen: Meter = 0.1 #Predefined length for bats

    def from_json(data):
        return Bat(data["colour"])

@dataclass
class Arm:
    comp: List[Tuple[Link,Joint]] #List of link-joint pairs. Should contain at least one pair.
    bat: Bat #The link representing the bat

    # Append the given link and joint to the arm
    def append(self, link: Link, joint: Joint):
        self.comp.append((link, joint))

    def from_json(data):
        joints = data["joints"]
        links = data["links"]

        comp = []
        for linkdata, jointdata in zip(links, joints):
            link = Link.from_json(linkdata)
            joint = Joint.from_json(jointdata)
            comp.append((link, joint))

        bat = Bat.from_json(links[-1])

        return Arm(comp, bat)
        
@dataclass
class Snapshot:
    time: Second # The current time
    ball: Pnt # Current position of the ball
    segment: Seg # Edge of some object in the room

    def from_json(data):
        time = parse_time(data["time"])
        ball = Pnt.from_json(data["ball"])
        segment = Seg.from_json(data["segment"])

        return Snapshot(time, ball, segment)


@dataclass
class Control: 
    accelerations: List[RadianPerSquareSecond] # List of joint accelerations

    def from_json(data):
        accelerations = [float(acc) for acc in data]
        return Control(accelerations)

@dataclass
class BallState:
    location: Pnt # Position of the ball
    direction: Vec # Velocity of the ball

    def from_json(data):
        data_location = data["location"]
        data_direction = data["direction"]

        location = Pnt.from_json(data_location)
        direction = Vec.from_json(data_direction)

        return BallState(location, direction)

class Item(IntEnum):
    AIR = 0
    BAT_SELF = 1
    BAT_OPPONENT = 2
    TABLE_SELF = 3
    TABLE_OPPONENT = 4
    NET = 5
    OTHER = 6

    def from_json(data):
        if data >= 100:
            return Item.OTHER
        else:
            return Item(data)
