module PingPong.Simulation where

import PingPong.Model
import PingPong.Draw
import PingPong.Simulation.Collision
import PingPong.Simulation.ForwardKinematics hiding (evaluateArm, controlArm)
import PingPong.Submission hiding (name, arm, dance, action)

import Data.Geometry hiding (init, head, zero, replicate)
import Data.Geometry.Matrix
import Data.Geometry.Transformation

import Data.Fixed
import Data.Ext
import Data.List hiding (intersect)
import Data.Foldable
import Control.Lens hiding (snoc)

import Convert

import Data.Vinyl.CoRec

import Debug.Trace

import System.Random
import System.Process
import Control.Concurrent


validatePlayer :: Player -> IO ()
validatePlayer p | validateArm $ arm p = return ()
                 | otherwise = error $ "Player " ++ name p ++ " has an invalid arm!"
  -- return something sensible so an invalid arm does not stop a tournament?


serveBall :: Bool -> IO BallState
serveBall p = do
  h <- randomRIO (0.9, 1)
  x <- randomRIO (1.3, 1.4)
  y <- randomRIO (-0.1, 0)
  return $ BallState (Point2 0 h) (Vector2 (if p then x else -x) y)


-- Duration of non-rally game phases in seconds
beforeGameTime  = 4
beforeRallyTime = 4
afterRallyTime  = 2
afterGameTime   = 10

-- updating

getPlayer :: Bool -> State -> Player
getPlayer True  = p1
getPlayer False = p2

act :: Bool -> State -> IO Control
act b st = case phase st of BeforeGame _  -> stretch (getPlayer b st) (time st) $ arm $ getPlayer b st
                            BeforeRally _ -> straightenUp b st
                            DuringRally   -> actDuringRally b st
                            AfterRally _  -> actDuringRally b st
                            AfterGame _   -> dance (getPlayer b st) (time st) $ arm $ getPlayer b st
                            _             -> return $ replicate 5 0

-- make this smarter, taking current velocities into account

straightenUp :: Bool -> State -> IO Control
straightenUp True  st = return $ straighten (p1 st)
straightenUp False st = return $ straighten (p2 st)

straighten :: Player -> Control
straighten p = 
  let js = armJoints $ arm p 
      gs = armJoints $ initArm p 
  in zipWith straightenJoint js gs

modAngle :: Float -> Float
modAngle x = (x + pi) `mod'` (2 * pi) - pi

straightenJoint :: Joint -> Joint -> RadianPerSquareSecond
straightenJoint (Joint _ u0 v0) (Joint _ u1 v1) = planMotion (modAngle u0) v0 (modAngle u1) v1

planMotion :: Radian -> RadianPerSecond -> Radian -> RadianPerSecond -> RadianPerSquareSecond
planMotion u0 v0 u1 v1 = 6 * u1 - 2 * v1 - 6 * u0 - 6 * v0



actDuringRally :: Bool -> State -> IO Control
actDuringRally True  st = action (p1 st) (time st) (head $ hits st) (ball st) (arm $ p1 st)
actDuringRally False st = act True (flipState st)
-- fmap flipMotion $
-- don't flip resulting motion -> motion is always in local perspective

-- | Main update function: advances time by a given amount.
update :: IOSubmission -> Float -> State -> IO State
update sub deltaTime st = do
  newcontrol1 <- act True  st
  newcontrol2 <- act False st
  let omb = dir $ ball st
  let initialTime  = time st
      goalTime     = initialTime + deltaTime
      initialState = st {c1 = newcontrol1, c2 = newcontrol2} -- , mb = omb}
  finalState <- case phase st of
                     BeforeRally _ -> updateUntilGhost (controlArmIO sub) goalTime initialState {frame = frame st + 1}
                     _ -> updateUntil sub goalTime initialState {frame = frame st + 1}
  curvedState <- curveBall deltaTime finalState
  perturbedState <- perturb deltaTime curvedState
  updatePhase deltaTime perturbedState

-- update the phase: count down timer, and if reached 0, take approriate action
-- if phase is DuringRally, then check the last thing that was hit.
updatePhase :: Float -> State -> IO State
updatePhase delta st = f $ phase st
  where 
    f (BeforeGame t)  | t > delta = return $ st {phase = BeforeGame  $ t - delta}
                      | otherwise = initBeforeRally st 
    f (BeforeRally t) | t > delta = return $ st {phase = BeforeRally $ t - delta}
                      | otherwise = initDuringRally st
    f (AfterRally t)  | t > delta = return $ st {phase = AfterRally  $ t - delta}
                      | otherwise = initBeforeRally st
    f (AfterGame t)   | t > delta = return $ st {phase = AfterGame   $ t - delta}
                      | otherwise = return $ st {phase = GameOver} -- the game is over, simulation should stop
    f DuringRally     | testScore (map snd $ hits st) (loc $ ball st) == Nothing = return $ st
                      | otherwise = updateScore (unJust $ testScore (map snd $ hits st) (loc $ ball st)) (score st) st

initBeforeGame :: State -> IO State
initBeforeGame st = return $ st { phase = BeforeGame beforeGameTime
                                , score = (0, 0)
                                , ball  = BallState (Point2 (-1) 0.6) (Vector2 0.4 1)
--                                , p1    = (p1 st) {initArm = arm (p1 st)}
--                                , p2    = (p2 st) {initArm = arm (p2 st)}
                                }

initBeforeRally :: State -> IO State
initBeforeRally st = do
  b <- serveBall True
  return $ st { phase = BeforeRally beforeRallyTime
              , ball  = b {dir = Vector2 0 0}
              }

initDuringRally :: State -> IO State
initDuringRally st = do
  let (i, j) = score st
      p      = (i + j) `mod` 4 < 2
  b <- serveBall p
  return $ st { phase = DuringRally
              , hits  = [(0, Bat $ if p then Opponent else Self)] 
              , ball  = (ball st) {dir = dir b}
              }

initAfterRally :: State -> IO State
initAfterRally st = return $ st {phase = AfterRally afterRallyTime}

initAfterGame :: State -> IO State
initAfterGame st = return $ st {phase = AfterGame  afterGameTime}

unJust (Just x) = x

testScore :: [Item] -> Point 2 Float -> Maybe Bool
testScore [] _ = Nothing
testScore [_] _ = Nothing
testScore (Table Self : Bat Opponent : _) _ = Nothing
testScore (Table Opponent : Bat Self : _) _ = Nothing
testScore (Bat Self : Table Self : _) _ = Nothing
testScore (Bat Opponent : Table Opponent : _) _ = Nothing
testScore (_ : Bat Opponent : _) (Point2 x y) | y > 0.5 && x > 0 && x < 1 = Just False
                                              | otherwise                 = Just True
testScore (_ : Bat Self : _) (Point2 x y) | y > 0.5 && x > -1 && x < 0 = Just True
                                          | otherwise                  = Just False
testScore (_ : Table Opponent : _) _ = Just True
testScore (_ : Table Self : _) _ = Just False
testScore (Other a : Other b : is) p = testScore (Other b : is) p
testScore _ _ = Nothing

updateScore :: Bool -> (Int, Int) -> State -> IO State
updateScore True  (a, b) st | a >= 10 && b <  a  = initAfterGame $ st {score = (a + 1, b)}
updateScore False (a, b) st | a <  b  && b >= 10 = initAfterGame $ st {score = (a, b + 1)}
updateScore True  (a, b) st = initAfterRally $ st {score = (a + 1, b)}
updateScore False (a, b) st = initAfterRally $ st {score = (a, b + 1)}

perturb :: Float -> State -> IO State
perturb deltaTime st = do
  dx <- randomRIO (-amount, amount)
  dy <- randomRIO (-amount, amount)
  return $ st {ball = (ball st) {dir = dir (ball st) ^+^ Vector2 dx dy}}
    where amount = 0.025 * deltaTime


curveBall :: Float -> State -> IO State
curveBall deltaTime st = return $ st {ball = (ball st) {dir = decay *^ dir (ball st) ^+^ 2 * deltaTime *^ down}}
  where decay = (1 - 0.05) ** deltaTime

-- | Update state using fixed motion until the goal time.
--   Will recurse until the next collision event is later than the goal time.
updateUntil :: IOSubmission -> Float -> State -> IO State
updateUntil sub deadline st0 | deadline == time st0 = return st0
                             | otherwise = do
  let checker = buildCollisionCheckerIO (detectCollisionIO sub) (handleCollisionIO sub)
  st1 <- updateUntilRaw sub deadline st0
  let t0  = time st0
      t1  = time st1
      b0  = loc $ ball st0
      b1  = loc $ ball st1
      collide (i, s0, s1) = collide' checker i (t0, b0, s0) (t1, b1, s1)
      repeated (t, i, _) = i /= Air && (fst $ head $ hits st0) >= t0 && i == (snd $ head $ hits st0)
  segs0 <- segmentsAt (evaluateArmIO sub) st0
  segs1 <- segmentsAt (evaluateArmIO sub) st1
  collisions <- mapM collide $ zip3 items segs0 segs1
  let candidates = sort $ filter (not . repeated) $ collisions
      (t, i, v) = head $ candidates ++ [(t1, Air, dir $ ball st1)]
  updatedRaw <- updateUntilRaw sub t st0 { ball = (ball st0) {dir = v}
                                         , hits = newHits (hits st0) (t, i)
                                         }
  updateUntil sub deadline updatedRaw

items :: [Item]
items = map item $ [0..]

item :: Int -> Item
item 0 = Bat Self
item 1 = Bat Opponent
item 2 = Table Self
item 3 = Table Opponent
item 10 = Net
item x = Other x

newHits :: [(Float, Item)] -> (Float, Item) -> [(Float, Item)]
newHits os (_, Air) = os
newHits os n        = n : os

-- | Updates state without checking for collisions.
updateUntilRaw :: IOSubmission -> Float -> State -> IO State
updateUntilRaw sub deadline st | deadline == time st = return st
                               | otherwise = do
  let f   = deadline - time st
      op1 = p1 st
      op2 = p2 st
      ob  = ball st
  new1 <- updateArm sub f (c1 st) op1
  new2 <- updateArm sub f (c2 st) op2
  let np1 = op1 {arm = new1}
      np2 = op2 {arm = new2}
      nb  = straightBallStep f ob
  return $ st { time = deadline
              , p1   = np1
              , p2   = np2
              , ball = nb
              }

-- | Updates state without checking for collisions.
updateUntilGhost :: Controller -> Float -> State -> IO State
updateUntilGhost co deadline st | deadline == time st = return st
                                | otherwise = do
  let f   = deadline - time st
      op1 = p1 st
      op2 = p2 st
      ob  = ball st
  new1 <- updateArmRaw co f (c1 st) op1
  new2 <- updateArmRaw co f (c2 st) op2               
  let np1 = op1 {arm = new1}
      np2 = op2 {arm = new2}
      nb  = ob
  return $ st { time = deadline
              , p1   = np1
              , p2   = np2
              , ball = nb
              }

-- | Applies control for given duration, and updates joint velocities and angles.
updateArm :: IOSubmission -> Second -> Control -> Player -> IO Arm
updateArm sub seconds force p = do
  let a = applyControl seconds force (arm p)
  advanceArm (evaluateArmIO sub) seconds p {arm = a}

updateArmRaw :: Controller -> Second -> Control -> Player -> IO Arm
updateArmRaw co seconds force p = co seconds force $ arm p


segmentsAt :: Evaluator -> State -> IO [Seg]
segmentsAt ev st = do
  a1 <- playerGeom ev True  $ p1 st
  a2 <- playerGeom ev False $ p2 st
  return $  (last . toList . edgeSegments) a1
         :  (last . toList . edgeSegments) a2
         :  listEdges table 
         ++ listEdges room
         ++ [net] 
         ++ (init . toList . edgeSegments) a1
         ++ (init . toList . edgeSegments) a2
 

playerGeom :: Evaluator -> Bool -> Player -> IO (PolyLine 2 () Float)
playerGeom ev b p = do
  geom <- evaluateP ev (arm p) 
  return $ playerTransform (foot p) b geom


playerTransform :: (IsTransformable g, NumType g ~ Float, Dimension g ~ 2) => Float -> Bool -> g -> g
playerTransform d True = translateBy $ Vector2 d 0
playerTransform d False = scaleBy (Vector2 (-1) 1) . translateBy (Vector2 d 0)

advanceArm :: Evaluator -> Second -> Player -> IO Arm 
advanceArm ev f p = do
  let na = advanceArmRaw f $ arm p
  segs <- armSegments ev p {arm = na} True
  if or [ intersectsExact s t
        | s <- filter (not . degenerate) segs
        , t <- listEdges table ++ listEdges room
        ]
  then return $ arm p -- see if we can still do partial motion?
  else return na



intersectsExact :: LineSegment 2 () Float -> LineSegment 2 () Float -> Bool
--intersectsExact s t | converf s `intersects` converf t = traceShow (s, t) $ True
--                    | otherwise                        = False
intersectsExact s t = converf s `intersects` converf t

converf :: LineSegment 2 () Float -> LineSegment 2 () Rational
converf = endPoints . core . traverse %~ realToFrac


degenerate :: Eq r => LineSegment 2 () r -> Bool
degenerate s = s ^. start . core == s ^. end . core


{-  
  let rs, rt :: LineSegment 2 () Rational
      rs = s & endPoints %~ (traverse %~ realToFrac)
      rt = t & endPoints %~ (traverse %~ realToFrac)
  in intersects rs rt
-}

-- needs to check:
-- * for too fast motion
-- * collision with table
-- * self-collision
-- * reaching over origin?



-- ball step
straightBallStep :: Float -> BallState -> BallState
straightBallStep f st = st { loc = loc st .+^ f *^ dir st }

simpleBallStep :: Float -> BallState -> BallState
simpleBallStep f st = st { loc = loc st .+^ f *^ dir st
                         , dir = decay *^ dir st ^+^ 2 * f *^ down
                         }
  where decay = (1 - 0.05) ** f


down :: Vector 2 Float
down = Vector2 0 (-1)



-- collisions

collide' :: CollisionChecker -> Item
         -> (Float, Point 2 Float, LineSegment 2 () Float) 
         -> (Float, Point 2 Float, LineSegment 2 () Float) 
         -> IO (Float, Item, Vector 2 Float)
collide' checker i (t0, b0, s0) (t1, b1, s1) = do
  check <- verify checker (t0, b0, s0) (t1, b1, s1)
  case check of Nothing        -> return (t1, Air, (b1 .-. b0) ^/ (t1 - t0))
                Just (t, p, v) -> return (t, i, v)



