from PPData import *
from typing import Optional, List, Tuple, Callable
import logging
import numpy as np
import math
from itertools import accumulate
import copy

# Change to adapt the level of ouput from the python server:
# Values are DEBUG, INFO, ERROR
LOGGING_LEVEL = logging.ERROR
EPS = 0.000001

NUMLINKS     = 4

BAT_COLOR    = "#341235"

LINK_COLORS  = ["#a111b2"]  *  NUMLINKS
LINK_LENGTHS = [0.45, 0.15, 0.15, 0.15]

JOINT_COLORS = ["#777777"] * NUMLINKS
JOINT_ANGLES = [-0.3, 0.2, 0.2, -0.1]

#Exercise 1
def name() -> str:
    return "ModelPythonB4"

def make_arm() -> Arm:
    arm = Arm([], Bat(BAT_COLOR))

    for i in range(NUMLINKS):
        link = Link(LINK_COLORS[i], LINK_LENGTHS[i])
        joint = Joint(JOINT_COLORS[i], JOINT_ANGLES[i])
        arm.append(link, joint)

    return arm

#Exercise 2
def detectCollision(snap1: Snapshot, snap2: Snapshot) -> Optional[Second]:
    return None

#Exercise 3
def controlArm(time: Second, control: Control, arm: Arm) -> Arm:
    return arm

def evaluateArm(arm: Arm) -> List[Pnt]:
    lengths = [link.llen for link, _ in arm.comp]
    pnts = [Pnt(0,0)]
    for l in lengths:
        pnts.append(pnts[-1] + Pnt(0,l))
    pnts.append(pnts[-1] + Pnt(0,0.1)) 
    return pnts

def dance(time: Second, arm: Arm) -> Control:
    return [   20 * math.sin (6.0 * time)
           ,  -20 * math.cos (6.0 * time)
           ,   20 * math.sin (6.0 * time)
           ,  -20 * math.cos (6.0 * time)
           ]

#Exercise 4
def handleCollision(snap1: Snapshot, snap2: Snapshot, time: Second) -> Tuple[Pnt, Vec]:
    collision, f = collisionPoint(snap1, snap2, time)
    v = collisionVelocity(snap1, snap2, time, f)
    
    return (collision, v)

def almostZero(number: float) -> bool:
    if abs(number) < EPS:
        return True
    return False

def between_zero_and_one(x: float):
    return x >= 0 and x <= 1

def computeSegParameter(xa: float, xb: float, xc: float, xd: float, ya: float, yb: float, yc:float, yd: float, t: float) -> Optional[float]:
    f = 0.0
    x_zero = almostZero(xa + xc * t)
    y_zero = almostZero(ya + yc * t)
    if x_zero and y_zero:
        f = None
    elif x_zero:
        f = (yd - yb * t) / (ya + yc * t)
    elif y_zero:
        f = (xd - xb * t) / (xa + xc * t)
    else:
        f1 = (xd - xb * t) / (xa + xc * t)
        f2 = (yd - yb * t) / (ya + yc * t)
        if between_zero_and_one(f1):
            f = f1
        else:
            f = f2

    return f

def getTFromTime(time: float, time0: float, time1: float):
    return (time - time0) / (time1 - time0)

def collisionPoint(snap1: Snapshot, snap2: Snapshot, time: Second) -> Tuple[Pnt, float]:
    xb0, yb0 = snap1.ball.x, snap1.ball.y
    xb1, yb1 = snap2.ball.x, snap2.ball.y
    
    c0, d0 = snap1.segment.p, snap1.segment.q
    xc0, yc0 = c0.x, c0.y
    xd0, yd0 = d0.x, d0.y

    c1, d1 = snap2.segment.p, snap2.segment.q
    xc1, yc1 = c1.x, c1.y
    xd1, yd1 = d1.x, d1.y

    xa = xd0 - xc0
    ya = yd0 - yc0
    xb = xb0 - xc0 + xc1 - xb1
    yb = yb0 - yc0 + yc1 - yb1 
    xc = xc0 - xd0 + xd1 - xc1
    yc = yc0 - yd0 + yd1 - yc1
    xd = xb0 - xc0
    yd = yb0 - yc0

    t = getTFromTime(time, snap1.time, snap2.time)
    f = computeSegParameter(xa, xb, xc, xd, ya, yb, yc, yd, t)
    p = origin + (1 - t) * (1 - f) * (c0 - origin) \
               + (1 - t) * f       * (d0 - origin) \
               + t       * (1 - f) * (c1 - origin) \
               + t       * f       * (d1 - origin)

    return (p, f)
        
def reflectVector(v, d):    
    a = angle(Vec(1, 0), d)
    transform = rotation(a) @ reflectvertical() @ rotation(-a)
    np_v = np.array([v.a, v.b, 0])
    reflected = np_v @ transform
    return Vec(reflected[0], reflected[1])


def collisionVelocity(snap1, snap2, time, f) -> Vec:
    t = getTFromTime(time, snap1.time, snap2.time)
    c0 = snap1.segment.p
    d0 = snap1.segment.q
    c1 = snap2.segment.p
    d1 = snap2.segment.q
    tdiff = snap2.time - snap1.time

    v_line =   ((1 - t) * (d0 - origin) + t * (d1 - origin)) \
             - ((1 - t) * (c0 - origin) + t * (c1 - origin))

    v_seg =   (((1 - f) * (c1 - origin) + f * (d1 - origin)) \
            - ((1 - f) * (c0 - origin) + f * (d0 - origin))) \
            / tdiff

    v_ball = (snap2.ball - snap1.ball) / tdiff
    v_diff = v_ball - v_seg
    v_ref = reflectVector(v_diff, v_line)

    res = v_ref + v_seg
    return res

#Exercise 5
def inverse(arm: Arm, seg: Seg) -> List[Radian]:
    return [0.0] * NUMLINKS

#Exercise 6
def plan(current_time: Second, arm: Arm, time_bound: Second, 
            seg: Seg, velocity: Vec) -> Control:
    return [0.0] * NUMLINKS
#Exercise 7
def action(time: Second, item: Item, arm: Arm, ball: BallState) -> Control:
    return [ -10 * math.sin (2.2 * time)
           , -10 * math.cos (2.3 * time)
           ,  10 * math.sin (2.4 * time)
           ,  10 * math.cos (2.5 * time)]
