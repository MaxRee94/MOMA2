import json

from PPData import *
import PPGeometry as Geom
#########################
# Comment out if you do not want to use numpy
from numpy import int64, int32
#########################

class PingPongJSONEncoder(json.JSONEncoder):
    def default(self, data):
        if isinstance(data, Link):
            return {
                "colour": data.lcol,
                "length": data.llen
            }
        elif isinstance(data, Joint):
            return {
                "colour": data.jcol,
                "angle": data.jang,
                "velocity": data.jvel
            }
        elif isinstance(data, Bat):
            return {
                "colour": data.lcol,
                "length": data.llen
            }
        elif isinstance(data, Arm):
            links = []
            joints = []
            for piece in data.comp:
                links.append(piece[0])
                joints.append(piece[1])

            links.append(data.bat)

            return {"links": links, "joints": joints}
        elif isinstance(data, Control):
            return {
                "accelerations": data.accelerations
            }
        elif isinstance(data, Geom.Pnt):
            return [data.x, data.y]
        elif isinstance(data, Geom.Vec):
            return [data.a, data.b]
        elif isinstance(data, Geom.Seg):
            return {
                "start": data.p,
                "end": data.q
            }
        #########################
        # Comment out if you do not want to use numpy
        elif isinstance(data, int64) or isinstance(data, int32):
            return int(data)
        #########################
        else:
            return super().default(data)

def parse_detect_collision(data):
    snap1 = Snapshot.from_json(data["snapshot1"])
    snap2 = Snapshot.from_json(data["snapshot2"])

    return snap1, snap2

def parse_control_arm(data):
    time = parse_time(data["time"])
    control = Control.from_json(data["control"])
    update = Arm.from_json(data["arm"])
    
    return time, control, update

def parse_evaluate_arm(data):
    update = Arm.from_json(data["arm"])
    return update

def parse_dance(data):
    time = parse_time(data["time"])
    arm = Arm.from_json(data["arm"])
    return time, arm

def parse_handle_collision(data):
    snap1 = Snapshot.from_json(data["snapshot1"])
    snap2 = Snapshot.from_json(data["snapshot2"])
    time = parse_time(data["time"])

    return snap1, snap2, time

def parse_plan(data):
    time = parse_time(data["current_time"])
    thearm = Arm.from_json(data["current_arm"])
    goaltime = parse_time(data["goal_time"])
    goalseg = Seg.from_json(data["goal_segment"])
    goalvel = Vec.from_json(data["goal_velocity"])

    return time, thearm, goaltime, goalseg, goalvel

def parse_action(data):
    time = parse_time(data["time"])
    item = Item.from_json(data["lasthit"])
    update = Arm.from_json(data["arm"])
    state = BallState.from_json(data["state"])

    return time, item, update, state

def parse_inverse(data):
    arm = Arm.from_json(data["arm"])    
    seg = Seg.from_json(data["segment"])

    return arm, seg