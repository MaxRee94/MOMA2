module PingPong.Simulation.InverseKinematics where

-- implementation of inverse kinematics 

import Data.Geometry hiding (init, zero)
import Data.Geometry.Matrix
import Data.Geometry.Transformation
import Data.Geometry.PolyLine
import Data.Geometry.Polygon

import Data.Ext
import Data.List hiding (filter, map, null)
import Data.Either
import Control.Lens

import Data.Colour
import Data.Fixed


import PingPong.Model
import PingPong.Simulation.ForwardKinematics



inverse :: Arm -> Seg -> Maybe [Radian]
inverse arm seg = error "inverse not implemented"
