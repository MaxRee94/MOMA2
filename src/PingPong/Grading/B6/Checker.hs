module PingPong.Grading.B6.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Grading.B6.Types
import PingPong.Grading.Types

import PingPong.Model hiding (score)
import PingPong.Model.AlmostEqual

import PingPong.Submission hiding (arm, score, evaluateArm, controlArm)
import PingPong.Simulation.Collision
import PingPong.Simulation.ForwardKinematics
import PingPong.Communication.Interface

import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import qualified Data.ByteString.Lazy as B
import Data.List
import Data.Aeson

import Control.Monad
import Control.Lens
import Control.Exception

import System.Directory
import System.Random

-- | Produce a list of all test cases to be used, and their corresponding point values
getTestCaseRefs :: IO [TestCaseRef]
getTestCaseRefs = do
  let dir = fileDir assignment
  files <- listDirectory dir
  cases <- mapM readTestCase $ map (dir ++) files
  return $ sort $ map (view _1) cases
  

getTestCase :: TestCaseRef -> IO TestCase
getTestCase ref = readTestCase $ filePath (asid ref) (tcid ref)

readTestCase :: FilePath -> IO TestCase
readTestCase path = do
  x <- B.readFile path
  let ecase = eitherDecode x
  case ecase of Left  m -> error $ "readTestCase failed: " ++ show m
                Right c -> return c

-- | Check a given submission against a single test case
checkSubmission :: IOSubmission -> TestCaseRef -> IO TestResult
checkSubmission sub ref = do
  let planner  = planIO     sub
      evaluator = evaluateArmIO sub -- replace by model evaluator for grading
  testcase@(_, i) <- getTestCase ref
  testPlan planner testcase

framerate = 50.0
step :: Second
step = 1 / framerate


executePlan :: (Second -> Arm -> Second -> Seg -> Vec -> IO Control) -> (Second, Arm, Second, Seg, Vec) -> IO [(Second, Arm)]
executePlan planner (t1, arm, t2, seg, vec) | t1 > t2 + 1  = return []
                                            | t1 > t2      = do
  let newArm = advanceArmRaw step arm  
  recurse <- executePlan planner (t1 + step, newArm, t2, seg, vec)
  return $ (t1, arm) : recurse                                              
                                            | otherwise    = do
  control <- planner t1 arm t2 seg vec
  let newArm = controlArm step control arm  
  recurse <- executePlan planner (t1 + step, newArm, t2, seg, vec)
  return $ (t1, arm) : recurse

armMotion :: Arm -> Motion
armMotion = map jvel . armJoints

evaluateSnap :: Seg -> Vec -> (Second, Arm) -> (Second, Float, Float)
evaluateSnap seg vec (time, arm) = 
  let pts = evaluateArm arm
      p   = last $ init pts
      q   = last pts
      sps = motionVelocity (armMotion arm) arm
      v   = 0.5 *^ (last $ init sps) ^+^ 0.5 *^ (last sps)
      de  = maximum [ norm (p .-. seg ^. start ^. core)
                    , norm (q .-. seg ^. end   ^. core)
                    ]
      ve  = norm $ vec ^-^ v
  in (time, de, ve)

scoreSnap :: Second -> (Second, Float, Float) -> Int
scoreSnap g (t, d, v) =
  let st | g ~= t            =  2
         | abs (g - t) < 0.1 =  1
         | abs (g - t) < 0.5 =  0
         | otherwise         = -1
      sd | d ~= 0            =  2
         | d < 0.01          =  1
         | d < 0.1           =  0
         | otherwise         = -1
      sv | v ~= 0            =  2
         | v < 0.01          =  1
         | d < 0.1           =  0
         | otherwise         = -1
  in minimum [5, st + sd + sv]

writeSeg :: Seg -> String
writeSeg s = writePnt (s ^. start . core) ++ "---" ++ writePnt (s ^. end . core)

writePnt :: Pnt -> String
writePnt (Point2 x y) = "(" ++ show x ++ "," ++ show y ++ ")"

writeVec :: Vec -> String
writeVec (Vector2 x y) = "(" ++ show x ++ "," ++ show y ++ ")"

writeSnap :: (Second, Arm) -> String
writeSnap (t, a) = 
  let pts = evaluateArm a
      p   = last $ init pts
      q   = last pts
      sps = motionVelocity (armMotion a) a
      v   = 0.5 *^ (last $ init sps) ^+^ 0.5 *^ (last sps)
  in "at time " ++ show t ++ " the bat was at " ++ writeSeg (OpenLineSegment (p :+ ()) (q :+ ()))
                          ++ " with velocity " ++ writeVec v  

testPlan :: (Second -> Arm -> Second -> Seg -> Vec -> IO Control) -> TestCase -> IO TestResult
testPlan planner (ref, (t1, arm, t2, seg, vec)) = catchExceptions $ do
  snaps <- executePlan planner (t1, arm, t2, seg, vec)
  let f snap = scoreSnap t2 $ evaluateSnap seg vec snap
      best   = maximumBy (\x y -> compare (f x) (f y)) snaps
      result = TestResult { trid    = tcid ref
                          , success = True
                          , score   = f best
                          , message = "closest match: " ++ writeSnap best
                          }              
  return result            

catchExceptions :: IO TestResult -> IO TestResult
catchExceptions = handle catchErrorCall
                . handle catchInterfaceException

catchInterfaceException :: InterfaceException -> IO TestResult
catchInterfaceException e = return $ failure { message = show e }

catchErrorCall :: ErrorCall -> IO TestResult
catchErrorCall e = return $ failure { message = show e }
