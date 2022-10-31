module PingPong.Simulation.Realtime where

import PingPong.Model
import PingPong.Draw.Gloss
import PingPong.Simulation
import PingPong.Simulation.Collision
import PingPong.Submission hiding (prepare, terminate)
import PingPong.Submission.ModelNativeSubmission

import System.Exit
import Control.Monad

import Codec.Picture( PixelRGBA8( .. ) )
import Codec.Picture.Types (createMutableImage)

import Graphics.Gloss (Display (InWindow), Picture, Color, white)
import Graphics.Gloss.Interface.IO.Simulate (simulateIO)
import Graphics.Gloss.Data.ViewPort

import Graphics.Text.TrueType( Font, loadFontFile )

import Data.Geometry

simulationRate :: Int
simulationRate = 50

windowDisplay :: Display
windowDisplay = InWindow "Window" (1600, 800) (100, 100)


-- | The main play function.
play :: Player -> Player -> IO ()
play = playWithSubmission $ nativeToIOSubmission PingPong.Submission.ModelNativeSubmission.submission

-- | Play with certain functionality replaced by that from the given submission.
--   In particular, collision checking / handling and forward kinematics.
playWithSubmission :: IOSubmission -> Player -> Player -> IO ()
playWithSubmission sub ip1 ip2 = do

  Right font <- loadFontFile "fonts/lato/Lato-Bold.ttf"  


  let pwhite   = PixelRGBA8 255 255 255 255
  image <- createMutableImage 1920 1080 pwhite

  -- clean up any running python processes
  -- prepareAll
--  terminate ip1 -- in case still running from previous run
--  terminate ip2 -- in case still running from previous run
  prepare ip1
  prepare ip2

  validatePlayer ip1
  validatePlayer ip2

  initialState <- initBeforeGame $ defState {p1 = ip1, p2 = ip2}
  simulateIO windowDisplay
             white
             simulationRate
             initialState
             (drawState (evaluateArmIO sub) font image)
             (realtimeUpdate sub)


realtimeUpdate :: IOSubmission -> ViewPort -> Float -> State -> IO State
realtimeUpdate sub p f os = do
  ns <- update sub f os
  when (phase ns == GameOver) $ endGame ns
  return ns
    
endGame :: State -> IO ()
endGame st = do
  putStrLn $ "ending game"
  exitSuccess

danceContest :: IOSubmission -> Player -> Player -> IO ()
danceContest sub ip1 ip2 = do
  Right font <- loadFontFile "fonts/lato/Lato-Bold.ttf"  
  let pwhite   = PixelRGBA8 255 255 255 255
  image <- createMutableImage 1920 1080 pwhite
  prepare ip1
  prepare ip2
  validatePlayer ip1
  validatePlayer ip2
  let initialState = defState { phase = AfterGame 60
                              , p1 = ip1, p2 = ip2
                              , score = (0, 0)
                              , ball  = BallState (Point2 (-1) 0.6) (Vector2 0.4 1)
                              }
  simulateIO windowDisplay
             white
             simulationRate
             initialState
             (drawState (evaluateArmIO sub) font image)
             (realtimeUpdate sub)
