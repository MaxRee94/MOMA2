module PingPong.Grading.B1.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Grading.B1.Types

import PingPong.Model
import PingPong.Grading.Types
import PingPong.Simulation.Collision

import qualified PingPong.Submission as S
import qualified PingPong.Submission.ExampleNativeSubmission as E

import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import Control.Monad

import System.Random

getTestCaseRefs :: IO [TestCaseRef]
getTestCaseRefs = return [TestCaseRef "B1" 1 1]

checkSubmission :: (String, Arm) -> S.IOSubmission -> TestCaseRef -> IO TestResult
checkSubmission (name, arm) sub ref
  | not $ validateArm arm       = return $ TestResult 1 0 True "communication works fine, but your arm does not satisfy the restrictions."
  | arm  == S.arm  E.submission = return $ TestResult 1 0 True "communication works fine, but you should change your player's arm!"
  | name == S.name E.submission = return $ TestResult 1 0 True "communication works fine, but you should change your player's name!"
  | name == "PythonDummy"       = return $ TestResult 1 0 True "communication works fine, but you should change your player's name!"
  | otherwise                   = return $ TestResult 1 1 True "congratulations, everything seems to work so far!"
