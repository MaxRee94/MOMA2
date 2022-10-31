module PingPong.Grading.B3.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Grading.B3.Types

import PingPong.Model hiding (score)
import PingPong.Model.AlmostEqual
import PingPong.Grading.Types
import PingPong.Submission
import PingPong.Communication.Interface

import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import Data.Aeson
import qualified Data.ByteString.Lazy as B
import Data.Colour
import Data.Colour.Names
import Data.List

import GHC.Utils.Misc
import Control.Monad

import System.Directory
import System.Random
import Control.Exception
import Control.Lens


-- | This Module tests the folling functions:
--   controlArmIO      :: Second -> Control -> Arm -> IO Arm
--   evaluateArmIO     :: Arm -> IO [Pnt]


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
  let controller = controlArmIO  sub
      evaluator  = evaluateArmIO sub
  testcase@(_, i, o) <- getTestCase ref
  case i of Left  _ -> testControl controller testcase
            Right _ -> testEval evaluator testcase




writeArm :: Arm -> String
writeArm (Extend l j a) = "link " ++ show (llen l) ++ " joint " ++ show (jang j) ++ " " ++ show (jvel j) ++ " " ++ writeArm a 
writeArm (End    l    ) = "bat"

testControl :: (Second -> Control -> Arm -> IO Arm) -> TestCase -> IO TestResult
testControl controller (ref, Left (s, c, a), Left correctAnswer) = catchExceptions $ do
  givenAnswer <- controller s c a
  let cJoints = armJoints correctAnswer
      gJoints = armJoints givenAnswer
      correct = cJoints ~= gJoints
      val = length cJoints
      sco | correct   = val
          | otherwise = length $ filter id $ zipWith (~=) gJoints cJoints
      mes | correct   = "correct result: " ++ writeArm givenAnswer
          | otherwise = "incorrect result: " ++ writeArm givenAnswer ++ " should have been " ++ writeArm correctAnswer      
      result = TestResult { trid    = tcid ref
                          , success = True
                          , score   = sco
                          , message = mes
                          }                          
  return result

writePnts :: [Pnt] -> String
writePnts []                  = "<empty list>"
writePnts [Point2 x y]        = show x ++ " " ++ show y
writePnts (Point2 x y : pnts) = show x ++ " " ++ show y ++ " / " ++ writePnts pnts

testEval :: (Arm -> IO [Pnt]) -> TestCase -> IO TestResult
testEval evaluator (ref, Right arm, Right correctAnswer) = catchExceptions $ do
  givenAnswer <- evaluator arm
  let correct = givenAnswer ~= correctAnswer
      sco = length $ filter id $ drop 2 $ zipWith (~=) givenAnswer correctAnswer
      mes | correct   = "correct result: " ++ writePnts givenAnswer
          | otherwise = "incorrect result: " ++ writePnts givenAnswer ++ " should have been " ++ writePnts correctAnswer      
      result = TestResult { trid    = tcid ref
                          , success = True
                          , score   = sco
                          , message = mes
                          }                          
  return result




catchExceptions :: IO TestResult -> IO TestResult
catchExceptions = handle catchErrorCall
                . handle catchInterfaceException

catchInterfaceException :: InterfaceException -> IO TestResult
catchInterfaceException e = return $ failure { message = show e }

catchErrorCall :: ErrorCall -> IO TestResult
catchErrorCall e = return $ failure { message = show e }
