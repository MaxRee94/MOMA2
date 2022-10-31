module PingPong.Grading.B2.Checker (getTestCaseRefs, checkSubmission) where

import PingPong.Grading.B2.Types

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

import Control.Monad
import Control.Lens

import System.Directory
import System.Random
import Control.Exception



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
  let detector = detectCollisionIO sub
  testcase <- getTestCase ref
  testCollision detector testcase

testCollision :: ((Second, Pnt, Seg) -> (Second, Pnt, Seg) -> IO (Maybe Second)) -> TestCase -> IO TestResult
testCollision detector (ref, (state1, state2), correctAnswer) = catchExceptions $ do
  givenAnswer <- detector state1 state2
  putStrLn $ show givenAnswer
  let correct = correctAnswer ~= givenAnswer
      sco | correct   = value ref
          | otherwise = 0
      mes | correct   = "correct result: " ++ writeCollisionOutput givenAnswer
          | otherwise = "incorrect result: " ++ writeCollisionOutput givenAnswer ++ " should have been " ++ writeCollisionOutput correctAnswer
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

writeCollisionOutput :: TestOutput -> String
writeCollisionOutput Nothing = "no collision"
writeCollisionOutput (Just t) = "collision at time " ++ show t

