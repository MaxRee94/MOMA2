module PingPong.Submission.ExampleNativeSubmission (submission) where

import PingPong.Model hiding (name, arm, dance, plan, action)
import PingPong.Player
import qualified PingPong.Submission as Submission

import PingPong.Simulation.Collision hiding (modelHandler, modelDetector)

import Control.Lens

import Data.Geometry hiding (replicate)
import Data.Geometry.Matrix
import Data.Ext
import Data.Fixed
import Data.List hiding (intersect)
import Data.Colour
import Data.Colour.Names


-- Collect all exercises in a single record.
submission = Submission.Submission
  { Submission.name            = name
  , Submission.arm             = arm
  , Submission.detectCollision = detectCollision
  , Submission.handleCollision = handleCollision
  , Submission.controlArm      = controlArm
  , Submission.evaluateArm     = evaluateArm
  , Submission.dance           = dance
  , Submission.inverse         = inverse
  , Submission.plan            = plan
  , Submission.action          = action
  }


-- FOR EXERCISE B1 --

name :: String
name = "Example Native Player"

arm  :: Arm
arm = link (gradient 0.1) 0.45 -* joint (gradient 0.2) (-0.3)
   *- link (gradient 0.3) 0.15 -* joint (gradient 0.4) ( 0.2)
   *- link (gradient 0.5) 0.15 -* joint (gradient 0.6) ( 0.2)
   *- link (gradient 0.7) 0.15 -* joint (gradient 0.8) (-0.1)
   *- bat  (gradient 0.9)

gradient :: Float -> Colour Float
gradient x = blend x forestgreen lawngreen

-- FOR EXERCISE B2 --

detectCollision :: Snapshot -> Snapshot -> Maybe Second
detectCollision _ _ = Nothing

-- FOR EXERCISE B3 --

controlArm :: Second -> Control -> Arm -> Arm
controlArm _ _ a = a

evaluateArm :: Arm -> [Point 2 Float]
evaluateArm = map (Point2 0) . scanl (+) 0 . map llen . armLinks

dance :: Second -> Arm -> Control
dance t _ = [  20 * sin (6 * t)
            , -20 * cos (5 * t)
            ,  20 * sin (4 * t)
            , -20 * cos (3 * t)
            ]

-- FOR EXERCISE B4 --

handleCollision :: Snapshot -> Snapshot -> Second -> (Pnt, Vec)
handleCollision (_, p1, _) (_, p2, _) _ = (p1, p2 .-. p1)

-- FOR EXERCISE B5 --

inverse :: Arm -> Seg -> Maybe [Radian]
inverse _ _ = Nothing

-- FOR EXERCISE B6 --

plan :: Second -> Arm -> Second -> Seg -> Vec -> Control
plan _ _ _ _ _ = replicate (length $ armJoints arm) 0

-- FOR EXERCISE B7 --

action :: Second -> Item -> Arm -> BallState -> Control
action time item arm ball = plan time arm goalTime goalSeg goalVec
  where
    goalTime = 8.7
    goalSeg  = OpenLineSegment (Point2 (-0.3) 0.7 :+ ()) (Point2 (-0.3) 0.8 :+ ())
    goalVec  = Vector2 (-1) 0


