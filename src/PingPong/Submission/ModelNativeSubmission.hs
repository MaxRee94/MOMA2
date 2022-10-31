module PingPong.Submission.ModelNativeSubmission (submission) where

import PingPong.Model hiding (name, arm, dance, plan, action)
import PingPong.Model.AlmostEqual
import PingPong.Player
import qualified PingPong.Submission as Submission

import PingPong.Simulation.Collision hiding (modelHandler, modelDetector)

import Control.Lens

import Data.Geometry hiding (head, init, replicate)
import Data.Geometry.Matrix
import Data.Ext
import Data.List hiding (head, init, intersect)
import Data.Colour
import Data.Colour.Names
import Data.Fixed


import Debug.Trace

-- Collect all exercises in a single record.
submission = Submission.Submission
  { Submission.name            = name
  , Submission.arm             = arm
  , Submission.detectCollision = detectCollision
  , Submission.handleCollision = handleCollision
  , Submission.controlArm      = controlArm
  , Submission.evaluateArm     = evaluateArm
  , Submission.dance           = dance
  , Submission.inverse         = inverse
  , Submission.plan            = plan
  , Submission.action          = action
  }


---------------------
-- FOR EXERCISE B1 --
---------------------

name :: String
name = "Cor du Buy"

arm  :: Arm
arm = link black 0.3 -* joint black (-0.7)
   *- link black 0.3 -* joint black ( 1.2)
   *- link black 0.3 -* joint black ( 1.0)
   *- link black 0.3 -* joint black (-0.5)
   *- bat  red

---------------------
-- FOR EXERCISE B2 --
---------------------


detectCollision :: Snapshot -> Snapshot -> Maybe Second
detectCollision = modelDetector

-- | A potential collision is given by two real numbers: a time fraction,
--   and a fraction of the segment.
type PotentialCollision = (Float, Float)

-- | A potential collision is valid when both fractions are in the interval [0,1].
validCollision :: PotentialCollision -> Bool
validCollision (a, b) = a >= 0 && a <= 1 && b >= 0 && b <= 1

-- | The time at which a potential collision happens.
collisionTime :: Snapshot -> Snapshot -> PotentialCollision -> Second
collisionTime (t1, _, _) (t2, _, _) (u, _) = (1 - u) * t1 + u * t2

-- | The model detector.
modelDetector :: Snapshot -> Snapshot -> Maybe Second
modelDetector snap1 snap2 =
  let pocos = filter validCollision $ potentialCollisions snap1 snap2
  in case pocos of []      -> Nothing
                   (t : _) -> Just $ collisionTime snap1 snap2 t

-- | Collect all potential collisions. May return 0, 1, or 2 pairs of fractions.
--   Note that the snapshot times are irrelevant.
potentialCollisions :: Snapshot -> Snapshot -> [PotentialCollision]
potentialCollisions (_, b0, s0) (_, b1, s1) = 
  let c0 = s0 ^. start ^. core
      d0 = s0 ^. end   ^. core
      c1 = s1 ^. start ^. core
      d1 = s1 ^. end   ^. core
      Point2 xb0 yb0 = b0
      Point2 xb1 yb1 = b1
      Point2 xc0 yc0 = c0
      Point2 xc1 yc1 = c1
      Point2 xd0 yd0 = d0
      Point2 xd1 yd1 = d1
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
      us = solveQuadraticEquation k j i
      s u | almostZero $ xa + xc * u = (yd - yb * u) / (ya + yc * u)
          | almostZero $ ya + yc * u = (xd - xb * u) / (xa + xc * u)
          | otherwise = let s1 = (xd - xb * u) / (xa + xc * u)
                            s2 = (yd - yb * u) / (ya + yc * u)
                        in if 0 <= s1 && s1 <= 1 then s1 else s2 -- checkAlmostEqual s1 s2 
  in sort $ zip us $ map s us

-- | Solve equation of the form ax^2 + bx + c = 0.
--   Attempt at a somewhat robust implementation.
solveQuadraticEquation :: (Ord r, Enum r, Floating r, Show r) => r -> r -> r -> [r]
solveQuadraticEquation 0 0 0 = [0] -- [0..]
solveQuadraticEquation a 0 0 = [0]
solveQuadraticEquation 0 b 0 = [0]
solveQuadraticEquation 0 0 c = []
solveQuadraticEquation a b 0 = sort [0, -b / a]
solveQuadraticEquation a 0 c | (-c / a) <  0 = []
                             | (-c / a) == 0 = [0]
                             | (-c / a) >  0 = [sqrt (-c / a)]
solveQuadraticEquation 0 b c = [-c / b]
solveQuadraticEquation a b c | almostZero a || almostZero (a / b) || almostZero (a / c) = solveQuadraticEquation 0 b c
solveQuadraticEquation a b c = 
  let d = b^2 - 4 * a * c
      result | d == 0 = [-b / (2 * a)]
             | d >  0 = [(-b - sqrt d) / (2 * a), (-b + sqrt d) / (2 * a)]
             | otherwise = []
  in result

-- | Test whether a floating point number is zero, taking rounding errors into account.
almostZero :: (Floating r, Ord r) => r -> Bool
almostZero x = abs x < epsilon

-- | Treshold for rounding errors in zero tests.
epsilon :: Floating r => r
epsilon = 0.0001


---------------------
-- FOR EXERCISE B3 --
---------------------

controlArm :: Second -> Control -> Arm -> Arm
controlArm td c a = advanceArm td $ applyControl td c a

applyControl :: Second -> Control -> Arm -> Arm
applyControl td [] (End l) = End l
applyControl td (f : fs) (Extend l j a) = Extend l (controlJoint td f j) (applyControl td fs a)
applyControl td _ a = a -- wrong length of control vector

-- | Apply acceleration to a joint, taking into account maximum speed and acceleration.
controlJoint :: Second -> RadianPerSquareSecond -> Joint -> Joint
controlJoint td f (Joint c a v) = Joint c a (capSpeed (v + (capAcceleration f) * td))

advanceArm :: Second -> Arm -> Arm 
advanceArm td (End l)        = End l
advanceArm td (Extend l j a) = Extend l (advanceJoint td j) (advanceArm td a)

advanceJoint :: Second -> Joint -> Joint
advanceJoint td (Joint c a v) = Joint c (a + v * td) v





-- | Give the vertices of the arm in a list, including the base and the tip.
evaluateArm :: Arm -> [Point 2 Float]
evaluateArm arm = 
  let uniquePoints = nub $ evaluateArmElements arm
      points = case validateArmPoints arm uniquePoints of Just m  -> error $ "evaluateArm: " ++ m
                                                          Nothing -> uniquePoints
  in points             

validateArmPoints :: Arm -> [Pnt] -> Maybe String
validateArmPoints arm ps = 
  let links = map llen $ armLinks arm
      dists = map norm $ zipWith (.-.) (tail ps) (init ps)
  in if links ~= dists then Nothing
                       else Just $ show links ++ " is not the same as " ++ show dists



-- | Give the vertices of all arm elements. This will include duplicate vertices,
--   since the joint transformations are rotations and do not change the location.
evaluateArmElements :: Arm -> [Point 2 Float]
evaluateArmElements arm = map (origin .+^) 
                        $ map (flip transformBy $ Vector2 0 0) 
                        $ globalize $ transformations arm

transformations :: Arm -> [Transformation 2 Float]
transformations = map transformation . armElements

transformation :: Element -> Transformation 2 Float
transformation (Left  (Link  _ d  )) = translation $ Vector2 0 d
transformation (Right (Joint _ a _)) = rotation a

-- | Turns a sequence of local transformations into independent global transformations.
globalize :: [Transformation 2 Float] -> [Transformation 2 Float]
globalize ts = scanl (|.|) identity ts

dance :: Second -> Arm -> Control
dance t _ = [ -20 * sin (3 * t)
            ,  20 * cos (3 * t)
            , -20 * sin (3 * t)
            ,  20 * cos (3 * t)
            ]

---------------------
-- FOR EXERCISE B4 --
---------------------

handleCollision :: Snapshot -> Snapshot -> Second -> (Pnt, Vec)
handleCollision = modelHandler


modelHandler :: Snapshot -> Snapshot -> Second -> (Pnt, Vec)
modelHandler snap1 snap2 t = 
  let (p, s) = collisionPoint snap1 snap2 t
      v      = collisionVelocity snap1 snap2 t (p, s)
  in (p, v)


-- | For a given collision time, compute the corresponding point in space, and also 
--   report the fraction of the segment where the collision happens.
collisionPoint :: Snapshot -> Snapshot -> Second -> (Pnt, Float)
collisionPoint (t0, b0, seg0) (t1, b1, seg1) t =
  let c0 = seg0 ^. start ^. core
      d0 = seg0 ^. end   ^. core
      c1 = seg1 ^. start ^. core
      d1 = seg1 ^. end   ^. core
      Point2 xb0 yb0 = b0
      Point2 xb1 yb1 = b1
      Point2 xc0 yc0 = c0
      Point2 xc1 yc1 = c1
      Point2 xd0 yd0 = d0
      Point2 xd1 yd1 = d1
      xa = xd0 - xc0
      ya = yd0 - yc0
      xb = xb0 - xc0 + xc1 - xb1
      yb = yb0 - yc0 + yc1 - yb1 
      xc = xc0 - xd0 + xd1 - xc1
      yc = yc0 - yd0 + yd1 - yc1
      xd = xb0 - xc0
      yd = yb0 - yc0
      u = (t - t0) / (t1 - t0)
      p = origin .+^ (1-u) *^ (b0 .-. origin) 
                 .+^ u     *^ (b1 .-. origin)
      s | almostZero $ xa + xc * u = (yd - yb * u) / (ya + yc * u)
        | almostZero $ ya + yc * u = (xd - xb * u) / (xa + xc * u)
        | otherwise = let s1 = (xd - xb * u) / (xa + xc * u)
                          s2 = (yd - yb * u) / (ya + yc * u)
                      in if 0 <= s1 && s1 <= 1 then s1 else s2 -- checkAlmostEqual s1 s2 
  in (p, s)

checkAlmostEqual :: (Ord r, Floating r, Show r) => r -> r -> r
checkAlmostEqual a b | a == 0 && abs b > treshold = error message
                     | b == 0 && abs a > treshold = error message
                     | a == 0 || b == 0           = 0
                     | a / b > 1 + treshold       = error message
                     | b / a > 1 + treshold       = error message
                     | otherwise                  = a -- trace ("checking " ++ show a ++ " " ++ show b) a
  where treshold = 100
        message  = error $ "checkAlmostEqual: " ++ show a ++ " /= " ++ show b

-- | For a given collision time and corresponding point and fraction, compute the new velocity 
--   of the ball at that point and time.
collisionVelocity :: Snapshot -> Snapshot -> Second -> (Pnt, Float) -> Vec
collisionVelocity (t0, b0, seg0) (t1, b1, seg1) t (p, s) | t < t0 || t > t1 = error "collisionVelocity: time not in range"
collisionVelocity (t0, b0, seg0) (t1, b1, seg1) t (p, s) = 
  let u = (t - t0) / (t1 - t0)
      c0 = seg0 ^. start ^. core
      d0 = seg0 ^. end   ^. core
      c1 = seg1 ^. start ^. core
      d1 = seg1 ^. end   ^. core
      vl =   ((1-u) *^ (d0 .-. origin) ^+^ u *^ (d1 .-. origin)) 
         ^-^ ((1-u) *^ (c0 .-. origin) ^+^ u *^ (c1 .-. origin))
      vs =   (((1-s) *^ (c1 .-. origin) ^+^ s *^ (d1 .-. origin)) 
         ^-^ ((1-s) *^ (c0 .-. origin) ^+^ s *^ (d0 .-. origin)))
         ^/  (t1 - t0)
      vb = (b1 .-. b0) ^/ (t1 - t0)
      vr = vb ^-^ vs
      vm = reflectVector vr vl
  in vm ^+^ vs

-- | Reflect vector 'a' in a line with direction vector 'b'.
reflectVector :: Vector 2 Float -> Vector 2 Float -> Vector 2 Float
reflectVector a b = reflection (angle (Vector2 1 0) b) `transformBy` a

-- | Find the angle between two vectors, in counter-clockwise order, from the first to the second.
angle :: Vector 2 Float -> Vector 2 Float -> Float
angle (Vector2 x1 y1) (Vector2 x2 y2) = atan2 (x1 * y2 - y1 * x2) (x1 * x2 + y1 * y2)

-- A more general version of angle which is not correct?
angle' :: (Arity d, Floating r) => Vector d r -> Vector d r -> r
angle' u v = dot u v / norm u / norm v
















-- FOR EXERCISE B5 --

inverse :: Arm -> Seg -> Maybe [Radian]
inverse _ _ = Nothing

-- FOR EXERCISE B6 --

plan :: Second -> Arm -> Second -> Seg -> Vec -> Control
plan _ _ _ _ _ = replicate (length $ armJoints arm) 0

-- FOR EXERCISE B7 --

action :: Second -> Item -> Arm -> BallState -> Control
action time item arm ball = plan time arm goalTime goalSeg goalVec
  where
    goalTime = 8.7
    goalSeg  = OpenLineSegment (Point2 (-0.3) 0.7 :+ ()) (Point2 (-0.3) 0.8 :+ ())
    goalVec  = Vector2 (-1) 0

