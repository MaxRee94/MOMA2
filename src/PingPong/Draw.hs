module PingPong.Draw where

import Data.Colour
import Data.Colour.Names
import Data.Colour.SRGB
import Codec.Picture( PixelRGBA8( .. ), writePng )

convertColor :: Colour Float -> PixelRGBA8
convertColor c = 
  let RGB r g b = toSRGB c
      f x = fromInteger $ round $ 255 * x
  in  PixelRGBA8 (f r) (f g) (f b) (f 1)

cRoom, cTable, cNet, cBall :: Colour Float
cRoom  = sRGB 0.9 0.9 1.0 
cTable = sRGB 0.2 0.4 0.1 
cNet   = sRGB 0.2 0.2 0.2 
cBall  = orange  