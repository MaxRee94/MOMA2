module PingPong.Grading.B4.Types  where

import PingPong.Model
import PingPong.Grading.Types

import Data.Geometry hiding (init, head, replicate)

-- ASSIGNMENT NUMBER

assignment :: Assignment
assignment = "B4"

-- TEST CASES

type TestInput' = ((Second, Pnt, Seg), (Second, Pnt, Seg))
type TestInput  = ((Second, Pnt, Seg), (Second, Pnt, Seg), Second)
type TestOutput = (Point 2 Float, Vector 2 Float)
type TestCase   = (TestCaseRef, TestInput, TestOutput)
