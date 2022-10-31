{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE DeriveAnyClass #-}

module PingPong.Communication.Interface where


import PingPong.Model hiding (prepare, terminate)
import PingPong.Player
import PingPong.Submission
import PingPong.Simulation.Collision
import PingPong.Communication.JSON

import Network.Simple.TCP
import Network.Socket.ByteString as ByteString
import qualified Data.ByteString.UTF8 as BSUTF8
import qualified Data.ByteString as BS 
import qualified Data.ByteString.Lazy as BSL
import Data.Geometry
import Data.Ext
import Data.Aeson
import Data.Aeson.Types
import Data.Aeson.Encoding (encodingToLazyByteString)
import Data.Default

import System.Process
import Control.Concurrent
import Control.Exception

import Foreign.C
import Foreign.C.String

import Data.Colour
import Data.Colour.Names

-- If you are using the haskell-only version, comment the following lines and instead use:
-- callpython = undefined
foreign import ccall "PingPong/C/player.h callpython"
  callpython :: CString -> IO CString

type Port = Int

data InterfaceException = InterfaceException String deriving (Show, Exception)

-- | Given a unique and the filepath to a python source file,
--   create a submission. This does not start a python process yet.
readInterfaceSubmission :: FilePath -> IOSubmission
readInterfaceSubmission path = IOSubmission
  { isNative          = False
  , prepare           = interfacePrepare path
  , detectCollisionIO = interfaceDetectCollision path
  , handleCollisionIO = interfaceHandleCollision path
  , controlArmIO      = interfaceControlArm path
  , evaluateArmIO     = interfaceEvaluateArm path
  , danceIO           = interfaceDance path
  , inverseIO         = interfaceInverse path
  , planIO            = interfacePlan path
  , actionIO          = interfaceAction path
  }

gradient :: Float -> Colour Float
gradient x = blend x forestgreen lawngreen

ignoreIOExeption :: Default a => IO a -> IO a
ignoreIOExeption = handle ignoreExeption

ignoreExeption :: Default a => IOException -> IO a
ignoreExeption e = do
  putStrLn $ "IOException: " ++ show e
  return def

assumeNoErrorEither :: Either String a -> a
assumeNoErrorEither (Right x) = x
assumeNoErrorEither (Left  s) = throw $ InterfaceException s

interfaceCall errorHandler parser message = do
  let smessage = BSL.toStrict $ encodingToLazyByteString message
  canswer <- BS.useAsCString smessage callpython
  sanswer <- peekCString canswer
  let answer = BSUTF8.fromString $ sanswer
  let parsed = errorHandler $ parser $ assumeNoErrorEither $ eitherDecodeStrict answer
  return parsed
  
-- B1

interfacePrepare :: FilePath -> IO (String, Arm)
interfacePrepare path = do
  interfaceCall assumeNoErrorEither parseSetup $ pairs ("type" .= ("setup" :: String) <>
                                                          "module" .= (show path))
  -- B2

interfaceDetectCollision :: FilePath -> Snapshot -> Snapshot -> IO (Maybe Second)
interfaceDetectCollision path snap1 snap2 = 
  interfaceCall id parseCollisionDetection $ pairs ("type" .= ("collision_detection" :: String) <>
                                                         "module" .= (show path) <>
                                                         "snapshot1" .= (encodeSnapshot snap1) <>
                                                         "snapshot2" .= (encodeSnapshot snap2))

  -- B3

interfaceHandleCollision :: FilePath -> Snapshot -> Snapshot -> Second -> IO (Pnt, Vec)
interfaceHandleCollision path snap1 snap2 time = 
  interfaceCall assumeNoErrorEither parseCollisionHandling $ pairs ("type" .= ("collision_handling" :: String) <>
                                                        "module" .= (show path) <>
                                                        "snapshot1" .= (encodeSnapshot snap1) <>
                                                        "snapshot2" .= (encodeSnapshot snap2) <>
                                                        "time" .= time)
  
  -- B4

interfaceControlArm      :: FilePath -> Second -> Control -> Arm -> IO Arm
interfaceControlArm path time control arm = 
  interfaceCall assumeNoErrorEither parseControlArm $ pairs ("type" .= ("control_arm" :: String) <>
                                                 "module" .= (show path) <>
                                                 "time" .= time <>
                                                 "control" .= control <>
                                                 "arm" .= arm)

interfaceEvaluateArm     :: FilePath -> Arm -> IO [Pnt]
interfaceEvaluateArm path arm = 
  interfaceCall assumeNoErrorEither parseEvaluateArm $ pairs ("type" .= ("evaluate_arm" :: String) <>
                                                  "module" .= (show path) <>
                                                  "arm" .= arm)

interfaceDance           :: FilePath -> Second -> Arm -> IO Control
interfaceDance path time arm = 
  interfaceCall assumeNoErrorEither parseDance $ pairs ("type" .= ("dance" :: String) <>
                                            "module" .= (show path) <>
                                            "time" .= time <>
                                            "arm" .= arm)

  -- B5

interfaceInverse         :: FilePath -> Arm -> Seg -> IO (Maybe [Radian])
interfaceInverse path arm segment = 
  interfaceCall assumeNoErrorEither parseInverse $ pairs ("type" .= ("inverse" :: String) <> 
                                              "module" .= (show path) <>
                                              "arm" .= arm <> 
                                              "segment" .= segment)

  -- B6

interfacePlan            :: FilePath -> Second -> Arm -> Second -> Seg -> Vec -> IO Control
interfacePlan path current_time current_arm goal_time goal_segment goal_velocity = 
  interfaceCall assumeNoErrorEither parsePlan $ pairs ("type" .= ("plan" :: String) <> 
                                            "module" .= (show path) <>
                                            "current_time" .= current_time <>
                                            "current_arm" .= current_arm <>
                                            "goal_time" .= goal_time <>
                                            "goal_segment" .= goal_segment <> 
                                            "goal_velocity" .= goal_velocity)

  -- B7

interfaceAction          :: FilePath -> Second -> Item -> Arm -> BallState -> IO Control
interfaceAction path time item arm state = 
  interfaceCall assumeNoErrorEither parseAction $ pairs ("type" .= ("action" :: String) <> 
                                              "module" .= (show path) <>
                                              "time" .= time <> 
                                              "lasthit" .= item <>
                                              "arm" .= arm <> 
                                              "state" .= state)

