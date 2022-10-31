module PingPong.Model.AlmostEqual where

import PingPong.Model
import PingPong.Model.Parameters

import Data.Geometry
import Data.Ext

import Control.Lens

-- * Class and operator for testing whether all numbers in two
--   comparable objects are within a global treshold.


treshold :: Float
treshold = 0.001

class AlmostEq a where
  (~=) :: a -> a -> Bool

infix 4 ~=

instance AlmostEq Float where
  x ~= y = abs (x - y) < treshold 

instance AlmostEq Pnt where
  Point2 px py ~= Point2 qx qy = px ~= qx && py ~= qy

instance AlmostEq Vec where
  Vector2 px py ~= Vector2 qx qy = px ~= qx && py ~= qy

instance AlmostEq Seg where
  s1 ~= s2 = s1 ^. start . core ~= s2 ^. start . core && s1 ^. end . core ~= s2 ^. end . core

instance AlmostEq Joint where
  j1 ~= j2 = jang j1 ~= jang j2 && jvel j1 ~= jvel j2

instance AlmostEq a => AlmostEq (Maybe a) where
  Just x  ~= Just y  = x ~= y
  Nothing ~= Nothing = True
  _       ~= _       = False

instance AlmostEq a => AlmostEq [a] where
  xs ~= ys = length xs == length ys && and (zipWith (~=) xs ys)
