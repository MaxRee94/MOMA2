module PingPong.Grading.Types where

import PingPong.Model hiding (score)

import Data.Geometry hiding (init, head, replicate)
import Data.Ext
import Data.Aeson
import Control.Monad
import Control.Lens

import GHC.Generics

type Assignment = String -- B1 to B7.

reportsDir :: FilePath
reportsDir = "output/reports/"

testcaseDir :: FilePath
testcaseDir = "input/testcases/"

-- rename these:
fileDir :: Assignment -> FilePath
fileDir a = testcaseDir ++ a ++ "/"

filePath :: Assignment -> Int -> FilePath
filePath a i = fileDir a ++ show i ++ ".testcase"


data TestCaseRef = TestCaseRef
  { asid  :: Assignment
  , tcid  :: Int
  , value :: Int
  } deriving (Show, Eq, Ord, Generic)

instance ToJSON   TestCaseRef     where toEncoding = genericToEncoding defaultOptions
instance FromJSON TestCaseRef

data TestResult = TestResult
  { trid    :: Int
  , score   :: Int
  , success :: Bool
  , message :: String
  } deriving (Show, Eq, Ord, Generic)

failure :: TestResult
failure = TestResult
  { trid    = 0
  , score   = 0
  , success = False
  , message = ""
  }
