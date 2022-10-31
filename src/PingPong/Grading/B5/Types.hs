module PingPong.Grading.B5.Types  where

import PingPong.Model
import PingPong.Grading.Types

import Data.Geometry hiding (init, head, replicate)

-- ASSIGNMENT NUMBER

assignment :: Assignment
assignment = "B5"

-- TEST CASES

type TestInput' = Arm
type TestInput  = (Arm, Seg)
type TestCase   = (TestCaseRef, TestInput, Bool)
