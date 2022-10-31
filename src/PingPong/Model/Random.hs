module PingPong.Model.Random where

import PingPong.Model
import PingPong.Model.Parameters

import Data.Colour
import Data.Colour.SRGB

import System.Random

-- * Functionality to generate random instances of various types in Model.


-- these should probably use a class instance (Random or Uniform)

generateRandomArm :: IO Arm
generateRandomArm = do 
  n <- randomRIO (minNumberOfJoints, maxNumberOfJoints) :: IO Int
  c <- generateRandomColour
  extendArm n $ bat c

extendArm :: Int -> Arm -> IO Arm  
extendArm 0 a = return a
extendArm n a = do
  l <- generateRandomLink
  j <- generateRandomJoint
  extendArm (n - 1) $ Extend l j a

generateRandomJoint :: IO Joint
generateRandomJoint = do
  col <- generateRandomColour
  ang <- randomRIO (0, 2 * pi) :: IO Float
  vel <- randomRIO (-maxSpeed, maxSpeed) :: IO Float
  return Joint { jcol = col
               , jang = ang
               , jvel = vel
               } 

generateRandomLink :: IO Link
generateRandomLink = do
  col <- generateRandomColour
  len <- randomRIO (minLinkLength, maxLinkLength) :: IO Float
  return Link { lcol = col
              , llen = len
              }

generateRandomColour :: IO (Colour Float)
generateRandomColour = do
  r <- randomRIO (0, 1) :: IO Float
  g <- randomRIO (0, 1) :: IO Float
  b <- randomRIO (0, 1) :: IO Float
  return $ sRGB r g b 

