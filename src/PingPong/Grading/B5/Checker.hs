module PingPong.Grading.B5.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Grading.B5.Types

import PingPong.Model hiding (score)
import PingPong.Model.AlmostEqual
import PingPong.Grading.Types
import PingPong.Submission
import PingPong.Communication.Interface
import PingPong.Simulation.Collision

import Data.List
import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import Data.Aeson
import qualified Data.ByteString.Lazy as B
import Control.Monad


import System.Directory
import System.Random
import Control.Exception
import Control.Lens








-- | This Module tests the folling functions:
--   inverse :: Arm -> Seg -> Maybe [Radian]


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
  let inverter  = inverseIO     sub
      evaluator = evaluateArmIO sub -- replace by model evaluator for grading
  testcase@(_, i, o) <- getTestCase ref
  testInverse inverter evaluator testcase

writeAngles :: [Radian] -> String
writeAngles = show

writeSeg :: Seg -> String
writeSeg s = writePnt (s ^. start . core) ++ "---" ++ writePnt (s ^. end . core)

writePnt :: Pnt -> String
writePnt (Point2 x y) = "(" ++ show x ++ "," ++ show y ++ ")"

testInverse :: (Arm -> Seg -> IO (Maybe [Radian])) -> (Arm -> IO [Pnt]) -> TestCase -> IO TestResult
testInverse inverter evaluator (ref, (arm, seg), possible) = catchExceptions $ do
  givenAnswer <- inverter arm seg
  case givenAnswer of Nothing -> do let val | possible  = 3
                                            | otherwise = 2
                                        sco | possible  = 0
                                            | otherwise = val
                                        mes | possible  = "incorrect result: not possible should have been possible"
                                            | otherwise = "correct result: not possible"
                                        result = TestResult { trid    = tcid ref
                                                            , success = True
                                                            , score   = sco
                                                            , message = mes
                                                            }                          
                                    return result
                      Just rs -> if length rs /= length (armJoints arm) then return $ TestResult (tcid ref) 0 True $ "wrong number of angles: " ++ writeAngles rs ++ " for an arm with " ++ show (length $ armJoints arm) ++ " joints"
                            else do let newArm = applyJointAngles rs arm
                                    points <- evaluator newArm
                                    let newSeg = OpenLineSegment ((last . init) points :+ ()) (last points :+ ())
                                        correct = newSeg ~= seg
                                        val | possible  = 3
                                            | otherwise = 2
                                        sco | correct   = val
                                            | otherwise = 0
                                        mes | correct   = "correct result: " ++ writeAngles rs ++ " leads to " ++ writeSeg newSeg
                                            | otherwise = "incorrect result: " ++ writeAngles rs ++ " leads to " ++ writeSeg newSeg
                                                                                                 ++ " but should be " ++ writeSeg seg
                                        result = TestResult { trid    = tcid ref
                                                            , success = True
                                                            , score   = sco
                                                            , message = mes
                                                            }                          
                                    return result


applyJointAngles :: [Radian] -> Arm -> Arm 
applyJointAngles rs       (End l)        = End l
applyJointAngles (r : rs) (Extend l j a) = Extend l (applyJointAngle r j) (applyJointAngles rs a)

applyJointAngle :: Radian -> Joint -> Joint
applyJointAngle r (Joint c _ v) = Joint c r v



catchExceptions :: IO TestResult -> IO TestResult
catchExceptions = handle catchErrorCall
                . handle catchInterfaceException

catchInterfaceException :: InterfaceException -> IO TestResult
catchInterfaceException e = return $ failure { message = show e }

catchErrorCall :: ErrorCall -> IO TestResult
catchErrorCall e = return $ failure { message = show e }
