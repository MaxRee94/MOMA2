module PingPong.Grading.B7.Types  where

import PingPong.Model
import PingPong.Grading.Types

import Data.Geometry hiding (init, head, replicate)

-- ASSIGNMENT NUMBER

assignment :: Assignment
assignment = "B7"

-- TEST CASES

type TestInput  = ()
type TestOutput = ()
type TestCase   = (TestCaseRef, TestInput, TestOutput)
