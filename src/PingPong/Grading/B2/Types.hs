module PingPong.Grading.B2.Types  where

import PingPong.Model
import PingPong.Grading.Types

import Data.Geometry hiding (init, head, replicate)

-- ASSIGNMENT NUMBER

assignment :: Assignment
assignment = "B2"

-- TEST CASES

type TestInput  = ((Second, Pnt, Seg), (Second, Pnt, Seg))
type TestOutput = Maybe Second
type TestCase   = (TestCaseRef, TestInput, TestOutput)
