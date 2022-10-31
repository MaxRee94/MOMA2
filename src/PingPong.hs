module PingPong where

import           PingPong.Model
import           PingPong.Grading

-- import           PingPong.Simulation.Recording
import           PingPong.Simulation.Realtime
import           PingPong.Player

import           PingPong.Submission
import qualified PingPong.Submission.ExampleNativeSubmission as ExampleNativeSubmission
import qualified PingPong.Submission.ModelNativeSubmission as ModelNativeSubmission

import           PingPong.Communication.Interface
import           PingPong.Communication.JSON
import           Data.Aeson


-- If your player is implemented in Haskell use the following:

{-
main :: IO ()
main = do
  let submission = nativeToIOSubmission ModelNativeSubmission.submission

  p1 <- makePlayer submission
  p2 <- makePlayer $ nativeToIOSubmission $ ExampleNativeSubmission.submission

--  danceContest submission p1 p2
  play p1 p2
--  playWithSubmission submission p1 p2
--  gradeSubmission "B5" submission
  return ()
-}

-- If your player is implemented in Python use the following:


main :: IO ()
main = do
  let submission = readInterfaceSubmission "PongZillaB6"

  p1 <- makePlayer submission
  p2 <- makePlayer $ nativeToIOSubmission $ ExampleNativeSubmission.submission

--  danceContest submission p1 p2
--  play p1 p2
  playWithSubmission submission p1 p2
--  gradeSubmission "B5" submission
  return ()
