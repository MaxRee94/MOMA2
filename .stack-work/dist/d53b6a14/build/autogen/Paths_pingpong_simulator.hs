{-# LANGUAGE CPP #-}
{-# LANGUAGE NoRebindableSyntax #-}
{-# OPTIONS_GHC -fno-warn-missing-import-lists #-}
{-# OPTIONS_GHC -Wno-missing-safe-haskell-mode #-}
module Paths_pingpong_simulator (
    version,
    getBinDir, getLibDir, getDynLibDir, getDataDir, getLibexecDir,
    getDataFileName, getSysconfDir
  ) where

import qualified Control.Exception as Exception
import Data.Version (Version(..))
import System.Environment (getEnv)
import Prelude

#if defined(VERSION_base)

#if MIN_VERSION_base(4,0,0)
catchIO :: IO a -> (Exception.IOException -> IO a) -> IO a
#else
catchIO :: IO a -> (Exception.Exception -> IO a) -> IO a
#endif

#else
catchIO :: IO a -> (Exception.IOException -> IO a) -> IO a
#endif
catchIO = Exception.catch

version :: Version
version = Version [0,3,1,4] []
bindir, libdir, dynlibdir, datadir, libexecdir, sysconfdir :: FilePath

bindir     = "D:\\OneDrive\\Documenten\\Motion and Manipulation\\dev\\pingpong-simulator-0314\\.stack-work\\install\\985be220\\bin"
libdir     = "D:\\OneDrive\\Documenten\\Motion and Manipulation\\dev\\pingpong-simulator-0314\\.stack-work\\install\\985be220\\lib\\x86_64-windows-ghc-9.0.2\\pingpong-simulator-0.3.1.4-1j0NcknFNaTBa3mOyL04EA"
dynlibdir  = "D:\\OneDrive\\Documenten\\Motion and Manipulation\\dev\\pingpong-simulator-0314\\.stack-work\\install\\985be220\\lib\\x86_64-windows-ghc-9.0.2"
datadir    = "D:\\OneDrive\\Documenten\\Motion and Manipulation\\dev\\pingpong-simulator-0314\\.stack-work\\install\\985be220\\share\\x86_64-windows-ghc-9.0.2\\pingpong-simulator-0.3.1.4"
libexecdir = "D:\\OneDrive\\Documenten\\Motion and Manipulation\\dev\\pingpong-simulator-0314\\.stack-work\\install\\985be220\\libexec\\x86_64-windows-ghc-9.0.2\\pingpong-simulator-0.3.1.4"
sysconfdir = "D:\\OneDrive\\Documenten\\Motion and Manipulation\\dev\\pingpong-simulator-0314\\.stack-work\\install\\985be220\\etc"

getBinDir, getLibDir, getDynLibDir, getDataDir, getLibexecDir, getSysconfDir :: IO FilePath
getBinDir = catchIO (getEnv "pingpong_simulator_bindir") (\_ -> return bindir)
getLibDir = catchIO (getEnv "pingpong_simulator_libdir") (\_ -> return libdir)
getDynLibDir = catchIO (getEnv "pingpong_simulator_dynlibdir") (\_ -> return dynlibdir)
getDataDir = catchIO (getEnv "pingpong_simulator_datadir") (\_ -> return datadir)
getLibexecDir = catchIO (getEnv "pingpong_simulator_libexecdir") (\_ -> return libexecdir)
getSysconfDir = catchIO (getEnv "pingpong_simulator_sysconfdir") (\_ -> return sysconfdir)

getDataFileName :: FilePath -> IO FilePath
getDataFileName name = do
  dir <- getDataDir
  return (dir ++ "\\" ++ name)
