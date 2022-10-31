module PingPong.Grading.B3.Types  where

import PingPong.Model
import PingPong.Grading.Types

import Data.Geometry hiding (init, head, replicate)

-- ASSIGNMENT NUMBER

assignment :: Assignment
assignment = "B3"

-- TEST CASES

type ControlInput  = (Second, Control, Arm)
type ControlOutput = Arm
type EvalInput     = Arm
type EvalOutput    = [Pnt]
type TestInput     = Either ControlInput EvalInput
type TestOutput    = Either ControlOutput EvalOutput
type TestCase      = (TestCaseRef, TestInput, TestOutput)

