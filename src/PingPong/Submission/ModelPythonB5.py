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
EPS = 0.00001

NUMLINKS     = 4

BAT_COLOR    = "#341235"

LINK_COLORS  = ["#a111b2"]  *  NUMLINKS
LINK_LENGTHS = [0.45, 0.15, 0.15, 0.15]

JOINT_COLORS = ["#777777"] * NUMLINKS
JOINT_ANGLES = [-0.3, 0.2, 0.2, -0.1]

#Exercise 1
def name() -> str:
    return "ModelPythonB5"

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
    return (Pnt(0,0), Vec(0,0))

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
def plan(current_time: Second, arm: Arm, time_bound: Second, 
            seg: Seg, velocity: Vec) -> Control:
    return [0.0] * NUMLINKS
    
#Exercise 7
def action(time: Second, item: Item, arm: Arm, ball: BallState) -> Control:
    return [ -10 * math.sin (2.2 * time)
           , -10 * math.cos (2.3 * time)
           ,  10 * math.sin (2.4 * time)
           ,  10 * math.cos (2.5 * time)]
