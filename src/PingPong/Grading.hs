module PingPong.Grading where

import PingPong.Model hiding (score, prepare, terminate)
import PingPong.Model.AlmostEqual
import PingPong.Grading.Types
import PingPong.Grading.Reports
import PingPong.Grading.Assignments
import PingPong.Submission
import PingPong.Submissions (submissions)
import PingPong.Communication.Interface

import Data.List
import Data.Char
import Control.Monad
import Control.Lens
--import Control.Lens.Tuple
import Control.Exception

main :: IO ()
main = do

--  generateTestCases "B6"

  gradeAssignment "B6"

{-
  gradeAssignment "B1"
  gradeAssignment "B2"
  gradeAssignment "B3"
  gradeAssignment "B4"
-}

-- TODO: move to module Grading.Main

gradeAssignment :: Assignment -> IO ()
gradeAssignment i = do

  subs    <- submissions
  results <- mapM (gradeSubmission' i) subs

  makeSummary i $ zip subs results

gradeSubmission' :: Assignment -> IOSubmission -> IO [(TestCaseRef, TestResult)]
gradeSubmission' i sub = do
  cases   <- getTestCaseRefs i
  results <- gradeSubmission i sub
  return $ zip cases results

gradeSubmission :: Assignment -> IOSubmission -> IO [TestResult]
gradeSubmission i sub = handle catchIOException $ handle catchInterfaceException $ do
  (name, arm) <- prepare sub
  let han = handle (catchInterfaceExceptionWithName name)
          . handle (catchIOExceptionWithName name)
  putStrLn $ "checking submission " ++ name ++ " for assignment " ++ i
  han $ do
    cases   <- getTestCaseRefs i
    results <- han $ mapM (checkSubmission i (name, arm) sub) cases
    writeReport name i $ zip cases results
    -- terminate sub
    return results


catchInterfaceException :: InterfaceException -> IO [TestResult]
catchInterfaceException e = do
  putStrLn "unable to check submission (name unknown, prepare failed)"
  putStrLn $ show e
  return $ repeat failure {message = show e}

catchInterfaceExceptionWithName :: String -> InterfaceException -> IO [TestResult]
catchInterfaceExceptionWithName name e = do
  putStrLn $ "unable to check submission " ++ name
  putStrLn $ show e
  return $ repeat failure {message = show e}



catchIOException :: IOException -> IO [TestResult]
catchIOException e = do
  putStrLn "unable to check submission (name unknown, prepare failed)"
  putStrLn $ show e
  return $ repeat failure {message = show e}

catchIOExceptionWithName :: String -> IOException -> IO [TestResult]
catchIOExceptionWithName name e = do
  putStrLn $ "unable to check submission " ++ name
  putStrLn $ show e
  return $ repeat failure {message = show e}

