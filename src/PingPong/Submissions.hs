module PingPong.Submissions where

-- | This module collects all submissions.

import PingPong.Submission
import PingPong.Communication.Interface

import System.Directory
import System.FilePath

-- Native submissions must be explicitly listed, since they must be known at compile time.
import qualified PingPong.Submission.ExampleNativeSubmission as ExampleNativeSubmission
import qualified PingPong.Submission.ModelNativeSubmission as ModelNativeSubmission

submissions :: IO [IOSubmission]
submissions = do
  let ns = map nativeToIOSubmission nativeSubmissions
  fs <- foreignSubmissions
  return $ ns ++ fs






-- Native submissions must be explicitly listed, since they must be known at compile time.
nativeSubmissions :: [Submission]
nativeSubmissions = 
  [ ExampleNativeSubmission.submission
  , ModelNativeSubmission.submission
  ]





-- Foreign submissions are only detected at runtime. The contents of the Submission folder
-- are paired to unique port numbers.
-- Currently only supports Python.
foreignSubmissions :: IO [IOSubmission]
foreignSubmissions = do
  files <- listDirectory "src/PingPong/Submission/"
  let pythonFiles   = filter isPython files
      pythonModules = map dropExtension pythonFiles
  return $ map readInterfaceSubmission pythonModules

-- | Python submissions are expected to end in ".py", otherwise will be ignored.
isPython :: FilePath -> Bool
isPython path = takeExtension path == ".py"
