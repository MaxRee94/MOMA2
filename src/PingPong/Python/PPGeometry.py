from dataclasses import dataclass
#########################
# Comment out if you do not want to use numpy
import numpy as np
#########################
import math

@dataclass
class Pnt:
    x: float # x coordinate
    y: float # y coordinate

    def from_json(data):
        return Pnt(data[0], data[1])

    def __mul__(self, s):
        return Pnt(self.x * s, self.y * s)

    def __rmul__(self, s):
        return Pnt(self.x * s, self.y * s)
        
    def __add__(self, q):
        if isinstance(q, Pnt):
            return Vec(self.x + q.x, self.y + q.y)
        elif isinstance(q, Vec):
            return Pnt(self.x + q.a, self.y + q.b)

    def __sub__(self, q):
        if isinstance(q, Pnt):
            return Vec(self.x - q.x, self.y - q.y)
        elif isinstance(q, Vec):
            return Pnt(self.x - q.a, self.y - q.b)

    def __truediv__(self, s):
        return Pnt(self.a / s, self.b / s)

    def __eq__(self, p):
        return math.isclose(self.x, p.x) and math.isclose(self.y, p.y)

origin = Pnt(0, 0)

@dataclass
class Seg:
    p: Pnt # first end point of segment
    q: Pnt # second end point of segment

    def from_json(data):
        return Seg(Pnt.from_json(data["start"]), 
                   Pnt.from_json(data["end"]))

    def __eq__(self, s):
        return (self.p == s.p and self.q == s.q) or (self.q == s.p and self.p == s.q)

@dataclass
class Vec:
    a: float # first entry of vector
    b: float # second entry of vector

    def from_json(data):
        return Vec(data[0], data[1])

    def norm(self) -> float:
        return math.sqrt(self.a ** 2 + self.b ** 2)

    def __mul__(self, s):
        if isinstance(s, Vec):
            return self.a * s.a + self.b * s.b

        return Vec(self.a * s, self.b * s)

    def __rmul__(self, s):
        if isinstance(s, Vec):
            return self.a * s.a + self.b * s.b

        return Vec(self.a * s, self.b * s)

    def __add__(self, q):
        if isinstance(q, Pnt):
            return Pnt(self.a + q.x, self.b + q.y)
        elif isinstance(q, Vec):
            return Vec(self.a + q.a, self.b + q.b)
        
    def __sub__(self, q):
        if isinstance(q, Pnt):
            return Pnt(self.a - q.x, self.b - q.y)
        elif isinstance(q, Vec):
            return Vec(self.a - q.a, self.b - q.b)
        
    def __truediv__(self, s):
        return Vec(self.a / s, self.b / s)

    def __eq__(self, v):
        return math.isclose(self.a, v.a) and math.isclose(self.b, v.b)

def angle(u: Vec, v: Vec) -> float:
    return math.atan2(u.a * v.b - u.b * v.a, u.a * v.a + u.b * v.b)

#########################
# Comment out if you do not want to use numpy
def rotation(a: float) -> np.array:
    return np.array([
            [math.cos(a), math.sin(-a), 0],
            [math.sin(a), math.cos( a), 0],
            [0          , 0           , 1]
        ])

def translation(d: Vec) -> np.array:
    return np.array([
            [1, 0, d.a],
            [0, 1, d.b],
            [0, 0, 1]
        ])

def reflectvertical() -> np.array:
    return np.array([
                [1,  0, 0],
                [0, -1, 0],
                [0,  0, 1]
            ])

def identity() -> np.array:
    return np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])
#########################