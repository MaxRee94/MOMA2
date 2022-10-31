from PPData import *
from typing import Optional, List, Tuple, Callable
import logging
import numpy as np
import math
from itertools import accumulate

GRAVITY = 2.0
SPEED_CAP = 2.0
ACCEL_CAP = 5.0
# Change to adapt the level of ouput from the python server:
# Values are DEBUG, INFO, ERROR
LOGGING_LEVEL = logging.ERROR
EPS = 0.00001

def almost(number: float, to: float) -> bool:
    if abs(number) < EPS:
        return True
    return False

def cap(real: float, capval: float) -> float:
    realabs = abs(real)
    if almost(realabs - capval, 0) or realabs > capval:
        if real < 0:
            return -capval
        return capval

    return real

def capspeed(speed: float) -> float:
    return cap(speed, SPEED_CAP)

def capaccel(accel: float) -> float:
    return cap(accel, ACCEL_CAP)

#Exercise 1
def name() -> str:
    return "pyping"

def make_arm() -> Arm:
    arm = Arm([], Bat("#341235"))

    for i in range(1,6):
        link = Link("#a111b2", 0.2)
        joint = Joint(f"#{i}{i}{i}{i}{i}{i}", 0)
        arm.append(link, joint)

    return arm

#Exercise 2
def detectCollision(snap1: Snapshot, snap2: Snapshot) -> Optional[Second]:
    print("DETECT")
    print(snap1, snap2)
    pocos = potentialCollisions(snap1, snap2)
    time = None
    for poco in pocos:
        if validPoco(poco):
            time = collisionTime(snap1, snap2, poco[0])
            break

    return time

def between_zero_and_one(x: float):
    return x >= 0 and x <= 1

def validPoco(poco: List[float]):
    return between_zero_and_one(poco[0]) and between_zero_and_one(poco[1])

def collisionTime(snap1: Snapshot, snap2: Snapshot, t: float) -> Second:
    return (1 - t) * snap1.time + t * snap2.time

def computeSegParameter(xa: float, xb: float, xc: float, xd: float, ya: float, yb: float, yc:float, yd: float, t: float) -> Optional[float]:
    f = 0.0
    x_zero = almost(xa + xc * t, 0)
    y_zero = almost(ya + yc * t, 0)
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

def potentialCollisions(snap1: Snapshot, snap2: Snapshot) -> List[List[float]]:
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

    i = xd * ya - yd * xa
    j = xd * yc - xb * ya - yd * xc + yb * xa
    k = yb * xc - xb * yc

    ts = solve_quadratic_eq(k, j, i)
    fs = []
    for t in ts:
        f = computeSegParameter(xa, xb, xc, xd, ya, yb, yc, yd, t)
        if f != None:
            fs.append(f) 

    print(sorted(zip(ts, fs))) 
    return sorted(zip(ts, fs))


def solve_azero(a: float, b: float, c: float) -> List[float]:
    return [-c / b]

def solve_bzero(a: float, b: float, c: float) -> List[float]:
    val = -c / a
    val_sign = math.copysign(1.0, val)
    if val_sign < 0:
        return []
    else:
        return (math.sqrt(val))

def solve_czero(a: float, b: float, c: float) -> List[float]:
    sorted([0, -b / a])

def solve_nonzero(a: float, b: float, c: float) -> List[float]:
    d = (b * b) - 4 * a * c
    d_sign = math.copysign(1.0, d)
    # print(d_sign, d > 0, d)

    if almost(d, 0):
        return [-b / (2 * a)]
    elif d > 0:
        return [((-b) - math.sqrt(d)) / (2 * a), ((-b) + math.sqrt(d)) / (2 * a)]
    else:
        return []

def solve_quadratic_eq(a: float, b: float, c: float) -> List[float]:
    res = []
    a_zero = almost(a, 0)
    b_zero = almost(b, 0)
    c_zero = almost(c, 0)

    # print(a, b, c)
    # print(a_zero, b_zero, c_zero)
    # print(almost(a / b, 0.0), almost(a / c, 0.0))
    
    if c_zero and (a_zero or b_zero):
        res = [0] 
    elif a_zero and b_zero:
        res = []
    elif c_zero:
        res = solve_czero(a, b, c) 
    elif b_zero:
        res = solve_bzero(a, b, c)
    elif a_zero:
        res = solve_azero(a, b, c)
    elif almost(a / b, 0.0) or almost(a / c, 0.0):
        res = solve_quadratic_eq(0.0, b, c)
    else:
        res = solve_nonzero(a, b, c)

    return res

#Exercise 3
def controlArm(time: Second, control: Control, arm: Arm) -> Arm:
    apply_control(time, control, arm)
    advance_arm(time, arm)

    return arm

def apply_control(time: Second, control: Control, arm: Arm) -> Arm:
    for (_, joint), accel in zip(arm.comp, control.accelerations):
        control_joint(time, accel, joint)

def control_joint(time: Second, accel: RadianPerSquareSecond, joint: Joint):
    print(time, joint.jvel + capaccel(accel) * time)
    joint.jvel = capspeed(joint.jvel + capaccel(accel) * time)

def advance_arm(time: Second, arm: Arm):
    for _, joint in arm.comp:
        advance_joint(time, joint)

def advance_joint(time: Second, joint: Joint):
    joint.jang = joint.jang + joint.jvel * time

def evaluateArm(arm: Arm) -> List[Pnt]:
    ts = make_global(transformations(arm))
    vs = [t @ np.array([[0.0],[0.0],[1.0]]) for t in ts]
    vecs = [Vec(v[0][0], v[1][0]) for v in vs]
    pnts = [origin + vec for vec in vecs]
    single_pnts = list(dict.fromkeys(map(lambda p: (p.x,p.y), pnts)))
    pnts = [Pnt(p[0], p[1]) for p in single_pnts]

    return pnts

def make_global(ts: [np.array]):
    gts = list(accumulate(ts, np.matmul, initial=identity()))
    return gts

def transformations(arm: Arm) -> [np.array]:
    ts = []
    for link, joint in arm.comp:
        tlink, rjoint = transformation(link, joint)
        ts.append(tlink)
        ts.append(rjoint)

    ts.append(trans_link(arm.bat))

    return ts

def transformation(link: Link, joint: Joint) -> Tuple[np.array, np.array]:
    return trans_link(link), rot_joint(joint)

def trans_link(link: Link) -> np.array:
    return translation(Vec(0, link.llen))

def rot_joint(joint: Joint) -> np.array:
    return rotation(joint.jang)

def dance(time: Second, arm: Arm) -> Control:
    return [ -2 * math.sin(2.2 * time)
            , -2 * math.cos(2.3 * time)
            ,  2 * math.sin(2.4 * time)
            ,  2 * math.cos(2.5 * time)
            ]

#Exercise 4
def handleCollision(snap1: Snapshot, snap2: Snapshot, time: Second) -> Tuple[Pnt, Vec]:
    collision, f = collisionPoint(snap1, snap2, time)
    v = collisionVelocity(snap1, snap2, time, f)
    
    return (collision, v)

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
    return [0.0] * len(arm.comp) 

#Exercise 6
def plan(current_time: Second, arm: Arm, time_bound: Second, 
            seg: Seg, velocity: Vec) -> Control:
    return [0.0] * len(arm.comp) 

#Exercise 7
def action(time: Second, item: Item, arm: Arm, ball: BallState) -> Control:
    return plan(time, arm, 8.7, Seg(Pnt(-0.3, 0.7), Pnt(-0.3, 0.8)), Vec(-1.0, 0.0))


