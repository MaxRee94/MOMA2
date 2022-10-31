module PingPong.Simulation.Recording where

import PingPong.Model
import PingPong.Draw.Rasterific
import PingPong.Simulation
import PingPong.Simulation.Collision
import PingPong.Submission hiding (prepare, terminate, name)
import PingPong.Submission.ModelNativeSubmission

import Codec.Picture( PixelRGBA8( .. ), writePng )
import Graphics.Rasterific hiding (Drawing)
import Graphics.Rasterific.Texture
import Graphics.Text.TrueType( Font, loadFontFile )

import System.Directory
import System.Process
import System.IO

import Control.DeepSeq
import Data.Char


recDir :: String
recDir = "output/recordings/"


frameRate :: Float
frameRate = 50

frameCount :: Int
frameCount = 500

frameDuration :: Float
frameDuration = 1 / frameRate

-- | The main play function.
play :: Player -> Player -> IO ()
play = playWithSubmission $ nativeToIOSubmission PingPong.Submission.ModelNativeSubmission.submission


-- | Play with certain functionality replaced by that from the given submission.
--   In particular, collision checking / handling and forward kinematics.
playWithSubmission :: IOSubmission -> Player -> Player -> IO ()
playWithSubmission sub ip1 ip2 = do
  hSetBuffering stdout NoBuffering

--  terminate ip1 -- in case still running from previous run
--  terminate ip2 -- in case still running from previous run
  prepare ip1
  prepare ip2
  validatePlayer ip1
  validatePlayer ip2
  initialState <- initBeforeGame $ defState {p1 = ip1, p2 = ip2}
  putStrLn $ "[" ++ name ip1 ++ " versus " ++ name ip2 ++ "]"
  Right font <- loadFontFile "fonts/lato/Lato-Bold.ttf"  
  pics <- record sub frameCount font initialState
  export pics (filter isAlphaNum (name ip1) ++ "-" ++ filter isAlphaNum (name ip2))
  -- terminate ip1
  -- terminate ip2


record :: IOSubmission -> Int -> Font -> State -> IO [Drawing]
record sub 0 _    _  = return []
record sub _ _    st | phase st == GameOver = return []
record sub n font os = do
  (ns, pic) <- step sub font os
  pics      <- record sub (n - 1) font ns 
  return $ pic : pics

step :: IOSubmission -> Font -> State -> IO (State, Drawing)
step sub font os = do
  pic <- drawState (evaluateArmIO sub) font os
  ns <- update sub frameDuration os
  return (ns, pic)




export :: [Drawing] -> String -> IO ()
export pics matchName = do
  createDirectoryIfMissing True $ recDir ++ "frame/"
  putStr "recording"
  sequence_ $ zipWith exportFrame [1..] pics
  putStrLn "done!"
  callCommand $ "ffmpeg -y -r " ++ show frameRate ++ " -f image2 -s 1920x1080 -i " ++ recDir ++ "frame/%d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p " ++ recDir ++ matchName ++ ".mp4"
  removeDirectoryRecursive $ recDir ++ "frame/"

exportFrame :: Int -> Drawing -> IO ()
exportFrame i pic = do
  let white = PixelRGBA8 255 255 255 255
      img = renderDrawing 1920 1080 white pic
  putStr $ frameMark i
  writePng (recDir ++ "frame/" ++ show i ++ ".png") img

frameMark :: Int -> String
frameMark i | i `mod` (60 * round frameRate) == 0 = show $ i `div` round frameRate
            | i `mod` (10 * round frameRate) == 0 = "#" 
            | i `mod` round frameRate == 0 = "|" 
            | otherwise                    = "."





