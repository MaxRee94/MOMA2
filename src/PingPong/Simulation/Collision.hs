module PingPong.Simulation.Collision
  ( CollisionChecker
  , defaultCollisionChecker
  , verify
  , Snapshot
  , buildCollisionCheckerIO
  , CollisionDetectorIO
  , CollisionHandlerIO
  , modelDetector
  , modelHandler
  ) where

import PingPong.Model

import Data.Geometry hiding (init, head, zero, replicate)
import Data.List hiding (intersect)
import Data.Foldable
import Data.Ext

import Control.Lens

import Data.Geometry.Transformation

-- * Implementation of collision checking.


-- | A collision checker takes as input two time stamps, with for each time stamp
--   the location of a point and a segment.
type Snapshot = (Second, Pnt, Seg)


-- | A collision checker takes as input two snapshots.
--   Assuming linear motion between the two time stamps, the checker should test
--   if a collision between the point and the segment takes place.
--   If so, it should report the time of the collision, as well as location and
--   velocity of the point as a result of the collision.
type CollisionChecker = Snapshot
                     -> Snapshot
                     -> IO (Maybe (Second, Pnt, Vec))

-- | A Collision detector only detects if there is a collision, and if so, when.
type CollisionDetector   = Snapshot -> Snapshot ->     Maybe Second
type CollisionDetectorIO = Snapshot -> Snapshot -> IO (Maybe Second)

-- | A Collision handler handles a collision, provided the time of collision.
type CollisionHandler   = Snapshot -> Snapshot -> Second ->    (Pnt, Vec)
type CollisionHandlerIO = Snapshot -> Snapshot -> Second -> IO (Pnt, Vec)

buildCollisionChecker :: CollisionDetector -> CollisionHandler -> CollisionChecker
buildCollisionChecker detect handle snap1 snap2 =
  case detect snap1 snap2 of 
    Nothing -> return Nothing
    Just t  -> let (p, v) = handle snap1 snap2 t
               in return $ Just (t, p, v)

buildCollisionCheckerIO :: CollisionDetectorIO -> CollisionHandlerIO -> CollisionChecker
buildCollisionCheckerIO detect handle snap1 snap2 = do
  det <- detect snap1 snap2
  case det of
    Nothing -> return Nothing
    Just t  -> do (p, v) <- handle snap1 snap2 t
                  return $ Just (t, p, v)

-- The collision checker that will be used when running the simulator through the play function.
defaultCollisionChecker :: CollisionChecker
defaultCollisionChecker = modelChecker

-- Make sure a collision checker returns something with a sensible time stamp.
verify :: CollisionChecker -> CollisionChecker
verify checker st1 st2 = do
  result <- checker st1 st2
  return $ verifyResult st1 st2 result

verifyResult :: Snapshot
             -> Snapshot
             -> Maybe (Second, Pnt, Vec)
             -> Maybe (Second, Pnt, Vec)
verifyResult _ _ Nothing = Nothing
verifyResult (t1, p1, s1) (t2, p2, s2) (Just (t, p, v)) | t <= t1 = Nothing
                                                        | t >= t2 = Nothing
                                                        | otherwise = Just (t, p, v)

-- A simple collision checker which ignores the segment, and only checks for collision with the floor.
floorChecker :: CollisionChecker
floorChecker (t1, Point2 x1 y1, _) (t2, Point2 x2 y2, _)
  | y2 >= 0   = return Nothing
  | y1 == y2  = error "Ball was already under the floor!?"
  | otherwise = let tc = t1 + (t2 - t1) * y1 / (y1 - y2)
                    xc = x1 + (x2 - x1) * y1 / (y1 - y2)
                    yc = 0
                    dx = (x2 - x1) / (t2 - t1)
                    dy = (y1 - y2) / (t2 - t1)
                in return $ Just (tc, Point2 xc yc, Vector2 dx dy)
  
-- The model solution.
modelChecker :: CollisionChecker
modelChecker = buildCollisionChecker modelDetector modelHandler






-- DETECTION

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
  -- trace ("soving equation " ++ show a ++ "x^2 + " ++ show b ++ "x + " ++ show c ++ " = 0") $ result  


-- | Test whether a floating point number is zero, taking rounding errors into account.
almostZero :: (Floating r, Ord r) => r -> Bool
almostZero x = abs x < epsilon

-- | Treshold for rounding errors in zero tests
--   TODO: Should be different depending on the type.
epsilon :: Floating r => r
epsilon = 0.0001










-- HANDLING

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
      s | almostZero $ xa + xc * u = (yd - yb * u) / (ya + yc * u)
        | almostZero $ ya + yc * u = (xd - xb * u) / (xa + xc * u)
        | otherwise = let s1 = (xd - xb * u) / (xa + xc * u)
                          s2 = (yd - yb * u) / (ya + yc * u)
                      in if 0 <= s1 && s1 <= 1 then s1 else s2 -- checkAlmostEqual s1 s2 
      p = origin .+^ (1-u) * (1-s) *^ (c0 .-. origin) 
                 .+^ (1-u) * s     *^ (d0 .-. origin) 
                 .+^ u     * (1-s) *^ (c1 .-. origin)
                 .+^ u     * s     *^ (d1 .-. origin)
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
