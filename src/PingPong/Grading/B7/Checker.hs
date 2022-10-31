module PingPong.Grading.B7.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Grading.B7.Types

import PingPong.Grading.Types
import PingPong.Submission
import PingPong.Simulation.Collision
import PingPong.Communication.Interface

import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import Control.Monad

import System.Random
import Control.Exception

getTestCaseRefs :: IO [TestCaseRef]
getTestCaseRefs = return []

checkSubmission :: IOSubmission -> TestCaseRef -> IO TestResult
checkSubmission sub ref = undefined



catchExceptions :: IO TestResult -> IO TestResult
catchExceptions = handle catchErrorCall
                . handle catchInterfaceException

catchInterfaceException :: InterfaceException -> IO TestResult
catchInterfaceException e = return $ failure { message = show e }

catchErrorCall :: ErrorCall -> IO TestResult
catchErrorCall e = return $ failure { message = show e }
