module PingPong.Grading.Assignments where

import PingPong.Model
import PingPong.Submission
import PingPong.Grading.Types

import qualified PingPong.Grading.B1.Checker as C1
import qualified PingPong.Grading.B2.Checker as C2
import qualified PingPong.Grading.B3.Checker as C3
import qualified PingPong.Grading.B4.Checker as C4
import qualified PingPong.Grading.B5.Checker as C5
import qualified PingPong.Grading.B6.Checker as C6
import qualified PingPong.Grading.B7.Checker as C7

getTestCaseRefs :: Assignment -> IO [TestCaseRef]
getTestCaseRefs "B1" = C1.getTestCaseRefs
getTestCaseRefs "B2" = C2.getTestCaseRefs
getTestCaseRefs "B3" = C3.getTestCaseRefs
getTestCaseRefs "B4" = C4.getTestCaseRefs
getTestCaseRefs "B5" = C5.getTestCaseRefs
getTestCaseRefs "B6" = C6.getTestCaseRefs
getTestCaseRefs "B7" = C7.getTestCaseRefs
getTestCaseRefs i    = noAssignment i

checkSubmission :: Assignment -> (String, Arm) -> IOSubmission -> TestCaseRef -> IO TestResult
checkSubmission "B1" x = C1.checkSubmission x
checkSubmission "B2" _ = C2.checkSubmission
checkSubmission "B3" _ = C3.checkSubmission
checkSubmission "B4" _ = C4.checkSubmission
checkSubmission "B5" _ = C5.checkSubmission
checkSubmission "B6" _ = C6.checkSubmission
checkSubmission "B7" _ = C7.checkSubmission
checkSubmission i    _ = noAssignment i

noAssignment :: Assignment -> a
noAssignment i = error $ "Non-existing assignment: " ++ i 
