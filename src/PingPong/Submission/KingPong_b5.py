# Team KingPong:
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

EPS = 0.0001

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
    return (Pnt(0,0), Vec(0,0))

def magnitude(vec) -> float:
    if isinstance(vec, Pnt):
        vec = Vec(vec.x, vec.y)
    return vec * vec

def normalize(vec: Vec) -> Vec:
    if isinstance(vec, Pnt):
        vec = Vec(vec.x, vec.y)
    magn = magnitude(vec)
    return (1.0 / magn) * vec

def p2v(pnt: Pnt) -> Vec:
    if isinstance(pnt, Vec):
        return pnt
    return Vec(pnt.x, pnt.y)

def dist(v1, v2) -> float:
    if isinstance(v1, Pnt):
        v1 = Vec(v1.x, v1.y)
    if isinstance(v2, Pnt):
        v2 = Vec(v2.x, v2.y)
    
    return magnitude(v2-v1)

#Exercise 5
def inverse(arm: Arm, seg: Seg) -> List[Radian]:
    # Initialize target (last joint of manipulator) and tip (tip of the bat)
    target = p2v(seg.p)
    tip = p2v(seg.q)

    # Check if too far away
    joint1_to_target = target - Vec(0, LINK_LENGTHS[0])
    if magnitude(joint1_to_target) > get_max_armlength(arm) :
        print("too far")
        return None

    # Check if manipulator's reachable workspace has an inner radius (min_dist), and if so check whether
    # the distance between the target and the first joint is smaller than this inner radius.
    # In this case, the target is unreachable.
    # positions = evaluateArm(arm)
    # lengths = [link.llen for link, _ in arm.comp]
    # lengths.pop(0)
    # second_link = lengths.pop(0)
    # invertible_arm_length = sum(lengths)
    # min_dist = second_link - invertible_arm_length
    # if (min_dist > 0 and dist(target, positions[1]) < min_dist):
    #     print("too close")
    #     return None
    
    # Run approximation algorithm
    iter_count = 0
    while True:
        i = len(evaluateArm(arm)) - 3 # index corresponding to current joint position
        j = len(arm.comp) - 2 # index corresponding to current arm component
        for _ in range(len(arm.comp)):
            # Get positions of base, joints, and tip of the arm
            positions = evaluateArm(arm)

            # Get change in angle d_a to get current segment endpoint as close to target as possible
            vec_joint2bat = p2v(positions[-2] - positions[i])
            joint2target_direction = normalize(target - p2v(positions[i]))

            d_a = angle(vec_joint2bat, joint2target_direction)

            # Update angle of the current joint
            arm.comp[j][1].jang += d_a

            i -= 1
            j -= 1

        positions = evaluateArm(arm)
        
        if dist(positions[-2], target) < IK_PRECISION_MARGIN and dist(positions[-1], tip):
            break
        
        if iter_count > IK_MAX_ITERATIONS:
            positions = evaluateArm(arm)
            # print("max iters")
            return None
        
        iter_count += 1

    # Rotate bat to align with segment
    vec_bat2_curtip = normalize(positions[-1] - positions[-2])
    vec_bat2tip = normalize(tip - positions[-2])
    d_a = angle(vec_bat2_curtip, vec_bat2tip)
    arm.comp[-1][1].jang += d_a

    positions = evaluateArm(arm)
    # print("--RUN--")
    # print("dist bat: ", dist(positions[-2], target))
    # print("dist tip: ", dist(positions[-1], tip))

    # Get list of angles from arm
    angles = [tupel[1].jang for tupel in arm.comp]

    return angles

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

