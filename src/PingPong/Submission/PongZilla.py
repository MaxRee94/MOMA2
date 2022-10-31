# Team PongZilla:
# Nick Bast (1455788)
# Max van der Ree (6241638)

import logging
from typing import Optional, Tuple
from PPData import *
import math
import time
import random
import numpy as np
from itertools import accumulate

# Change to adapt the level of ouput from the python server:
# Values are DEBUG, INFO, ERROR
LOGGING_LEVEL = logging.ERROR

EPS = 0.000001

NUMLINKS     = 5

SPEED_CAP = 2.0
ACCEL_CAP = 5.0

BAT_COLOR    = "#341235"

LINK_COLORS  = ["#3333AA", "#BB22AA", "#999999", "#222222", "#DD3333"]
LINK_LENGTHS = [0.22]       * NUMLINKS

JOINT_COLORS = ["#7777CC", "#7777CC", "#999999", "#CC7777", "#CC7777"]
JOINT_ANGLES = [1, -0.7, -0.7, -0.6, 1]

maxSpeed = 2
maxAcceleration = 5

#Exercise 1
def name() -> str:
    return "PongZilla"

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
EPS = 0.00000000001

def almostZero(number: float) -> bool:
    if abs(number) < EPS:
        return True
    return False

def cap(real: float, capval: float) -> float:
    realabs = abs(real)
    if almostZero(realabs - capval) or realabs > capval:
        if real < 0:
            return -capval
        return capval

    return real

def get_max_armlength(arm: Arm) -> float:
    cap = 0.0
    for c in arm.comp:
        cap += c[0].llen
    
    cap -= arm.comp[0][0].llen

    return cap

def capspeed(speed: float) -> float:
    return cap(speed, SPEED_CAP)

def capaccel(accel: float) -> float:
    return cap(accel, ACCEL_CAP)

def controlArm(time: Second, control: Control, arm: Arm) -> Arm:
    # Go through all joints and apply the acceleration
    applyControl(time, control, arm)
    
    # Move the angle for each joint accordingly
    advanceArm(time, arm)

    return arm

def applyControl(time: Second, control: Control, arm: Arm) -> Arm:
    for (_, joint), accel in zip(arm.comp, control.accelerations):
        controlJoint(time, accel, joint)

def controlJoint(time: Second, accel: RadianPerSquareSecond, joint: Joint):
    joint.jvel = capspeed(joint.jvel + capaccel(accel) * time)

def advanceArm(time: Second, arm: Arm):
    for _, joint in arm.comp:
        advanceJoint(time, joint)

def advanceJoint(time: Second, joint: Joint):
    joint.jang = joint.jang + joint.jvel * time

def evaluateArm(arm: Arm) -> List[Pnt]:
    # Create transformation matrices
    ts = makeGlobal(transformations(arm))

    # Find every point starting from (0,0) (as (0,0,1) in homogenous coordinates)
    vs = [t @ np.array([[0.0],[0.0],[1.0]]) for t in ts]

    # Remove doubled entries via dict.fromkeys and store as tuples of floats, also implicitly transforms from homogenous coordinates to 2D
    single_pnts = list(dict.fromkeys(map(lambda v: (v[0][0], v[1][0]), vs)))

    # Make points from the tuples
    pnts = [Pnt(p[0], p[1]) for p in single_pnts]

    return pnts

def makeGlobal(ts: [np.array]):
    gts = list(accumulate(ts, np.matmul, initial=identity()))
    return gts

def transformations(arm: Arm) -> [np.array]:
    ts = []
    for link, joint in arm.comp:
        tlink, rjoint = transformation(link, joint)
        ts.append(tlink)
        ts.append(rjoint)

    ts.append(transLink(arm.bat))

    return ts

def transformation(link: Link, joint: Joint) -> Tuple[np.array, np.array]:
    return transLink(link), rotJoint(joint)

def transLink(link: Link) -> np.array:
    return translation(Vec(0, link.llen))

def rotJoint(joint: Joint) -> np.array:
    return rotation(joint.jang)

def dance(time: Second, arm: Arm) -> Control:
    return [  -20 * math.cos (6.0 * time)
           ,   20 * math.sin (6.0 * time)
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
ITERATIONS = 1000
from itertools import cycle

def angles(arm: Arm) -> List[Radian]:
    return [j.jang for _, j in arm.comp]

def inverse(arm: Arm, seg: Seg) -> Optional[List[Radian]]:
    original = copy.deepcopy(arm)
    possible = solveInverse(ITERATIONS, arm, seg)

    if possible:
        return angles(arm)
    else:
        arm = original
        possible = solveInverse(ITERATIONS, arm, Seg(seg.q, seg.p))
        if possible:
            return angles(arm)
    
    return None

def solveInverse(iterations: int, arm: Arm, seg: Seg) -> bool:
    possible = ccd(iterations, seg.p, arm)
    setFinalAngle(arm, seg.q)

    return possible

def alength(arm: Arm):
    return len(arm.comp)

def ithjoint(i: int, arm: Arm) -> Joint:
    if i - 1 >= 0 and i - 1 < alength(arm):
        return arm.comp[i - 1][1]
    
    raise IndexError("Not enough joints.")

def addRadian(s: Radian, t: Radian) -> Radian:
    return (s + t) % (2 * math.pi)

def ccd(iterations: int, goal: Pnt, arm: Arm) -> bool:
    round = 0
    for i in cycle(range(1, alength(arm) + 1)):
        joint = ithjoint(i, arm)
        positions = evaluateArm(arm)
        pivot = positions[-2]

        distance = (goal - pivot).norm()
        if distance < EPS:
            break
        elif round == iterations:
            return False

        step(joint, positions[i], pivot, goal)

        round = round + 1
    
    return True
        
def step(joint: Joint, jpos: Pnt, pivot: Pnt, goal: Pnt):
    diff = angle((pivot - jpos), (goal - jpos))
    frac = 0.999 * diff
    joint.jang = addRadian(joint.jang, frac)

def setFinalAngle(arm: Arm, goal: Pnt):
    positions = evaluateArm(arm)
    fjoint = ithjoint(alength(arm), arm)
    tip = positions[-1]
    piv = positions[-2]
    u = tip - piv
    v = goal - piv
    dif = angle(u, v)

    fjoint.jang = addRadian(fjoint.jang, dif)

#Exercise 6
def plan(current_time: Second, arm: Arm, time_bound: Second, seg: Seg, velocity: Vec) -> Control:
    # https://www.rosroboticslearning.com/jacobian

    # curangles = [joint.jang for _, joint in arm.comp]
    # dt = time_bound - current_time
    lengths = [link.llen for link, _ in arm.comp]
    lengths.append(arm.bat.llen/2)
    # velocity is in the middle of seg
    goalangles = inverse(arm, seg)
    goalangles.insert(0, 0.0)

    # Jacobian
    # https://robotacademy.net.au/lesson/velocity-of-3-joint-planar-robot-arm/
    J = [[0 for _ in range(len(lengths))] for _ in range(2)]
    if goalangles is not None:
        for i in range(len(lengths)):
            x = 0
            y = 0
            for j in range(len(lengths) - i):
                ang = 0
                for a in range(len(lengths) - j):
                    ang += goalangles[a]
                x -= lengths[len(lengths)-j-1] * math.sin(ang)
                y += lengths[len(lengths)-j-1] * math.cos(ang)
            J[0][i] = x
            J[1][i] = y

    Jinv = np.linalg.pinv(J,EPS)
    v = np.array([velocity.a, velocity.b])
    # angle velocities
    Vang = Jinv * v

    ### -------- POLYNOMIAL FITTING - CUBIC FITTING -------- ###
    # 4 givens per joint (angle and ang_velocity for current_time and bound_time)
    # Therefore, polynomial should be cubic, i.e. ang(t) = at^3 + bt^2 + ct + d
    # with velocity the first derivative, i.e. vel(t) = 3at^2 + 2bt + c
    # and angular acceleration the second derivative, i.e. acc(t) = 6at + 2b
    tc = current_time
    tb = time_bound
    A = np.array(
        [
            [tc^3, tc^2, tc, 1],
            [tb^3, tb^2, tb, 1],
            [3*tc^2, 2*tc, 1, 0],
            [3*tb^2, 2*tb, 1, 0]
        ]
    )
    A_inv = np.linalg.inv(A)
    control = Control()
    for i, target_vel in enumerate(list(Vang)):
        # Construct vector of givens for the current joint
        start_ang = arm.comp[i][1].jang
        start_vel = arm.comp[i][1].jvel
        Y = np.array([start_ang, goalangles[i], start_vel, target_vel])

        # Solve the set of 4 linear equations and extract a and b
        X = A_inv.dot(Y)
        a = X[0]
        b = X[1]

        # Compute acceleration for current joint using a and b, and append to control
        print("acceleration: ", angular_acceleration(current_time, a, b))
        control.accelerations.append(angular_acceleration(current_time, a, b))

    return control

def angular_acceleration(t: Second, a: float, b: float) -> float:
    return 6.0 * a * t + 2.0 * b

#Exercise 7
def action ( time : Second , item : Item , arm : Arm , ball : BallState ) -> Control :
    goalTime = 8.7
    goalSeg = Seg ( Pnt ( -0.3 , 0.7) , Pnt ( -0.3 , 0.8))
    goalVel = Vec ( -1 , 0)
    return plan ( time , arm , goalTime , goalSeg , goalVel )

