module PingPong.Grading.B4.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Model hiding (score)
import PingPong.Model.AlmostEqual
import PingPong.Grading.Types
import PingPong.Submission
import PingPong.Communication.Interface

import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import Data.List
import Data.Aeson
import qualified Data.ByteString.Lazy as B

import System.Directory
import System.Random

import Control.Monad
import Control.Lens
import Control.Exception


-- ASSIGNMENT NUMBER

assignment :: Assignment
assignment = "B4"

-- TEST CASES

type TestInput  = ((Second, Pnt, Seg), (Second, Pnt, Seg), Second)
type TestOutput = (Point 2 Float, Vector 2 Float)
type TestCase   = (TestCaseRef, TestInput, TestOutput)



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
  let handler = handleCollisionIO sub
  testcase <- getTestCase ref
  testCollision handler testcase


writeCollisionOutput :: TestOutput -> String
writeCollisionOutput (p, v) = "collision at location " ++ show p ++ " resulting in direction " ++ show v



testCollision :: ((Second, Pnt, Seg) -> (Second, Pnt, Seg) -> Second -> IO (Pnt, Vec)) -> TestCase -> IO TestResult
testCollision handler (ref, (state1, state2, t), correctAnswer) = catchExceptions $ do
  givenAnswer <- handler state1 state2 t
  let correctPnt = fst correctAnswer ~= fst givenAnswer
      correctVec = snd correctAnswer ~= snd givenAnswer
      sco | correctPnt && correctVec = 2
          | correctPnt               = 1
          | correctVec               = 1
          | otherwise                = 0
      mes | correctPnt && correctVec = "correct result: " ++ writeCollisionOutput givenAnswer
          | otherwise                = "incorrect result: " ++ writeCollisionOutput givenAnswer ++ " should have been " ++ writeCollisionOutput correctAnswer
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
