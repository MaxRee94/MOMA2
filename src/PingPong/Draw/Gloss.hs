module PingPong.Draw.Gloss where

import PingPong.Model
import PingPong.Draw
import PingPong.Draw.Rasterific (renderState)

import Data.Colour
import Data.Colour.SRGB
import Graphics.Gloss (Picture, Color)
import Graphics.Gloss.Juicy
import qualified Graphics.Gloss as G

import Codec.Picture (PixelRGBA8 (..))
import Codec.Picture.Types (MutableImage)

import Graphics.Text.TrueType( Font, loadFontFile )

import Data.Geometry hiding (head, init)

import Convert
import PingPong.Simulation.ForwardKinematics

-- implement this via rasterific and printing image to screen?

--drawState :: Font -> MutableImage (PrimState IO) PixelRGBA8 -> State -> Picture
--drawState font image state = fromImageRGBA8 image -- . renderState font

--drawState font image = fromImageRGBA8 . renderState font


--drawState :: Evaluator -> Font -> State -> IO Picture
drawState ev font _ m = do
  pic1 <- drawPlayer ev (p1 m) True
  pic2 <- drawPlayer ev (p2 m) False
  return $ G.Pictures 
    [ center $ G.Pictures [drawStatic, pic1, pic2, drawBall $ ball m]
    , drawInfo m
    , drawPhase (phase m) m
    ]

drawInfo :: State -> Picture
drawInfo s = G.Pictures
  [ G.Translate (-580) (320) $ G.Scale 0.1 0.1 $ G.text $ (name $ p2 $ s) ++ " VS " ++ (name $ p1 $ s)
  , G.Translate (-580) (300) $ G.Scale 0.1 0.1 $ G.text $ "score: " ++ (show $ snd $ score s) ++ " - " ++ (show $ fst $ score s)
  , G.Translate (-580) (280) $ G.Scale 0.1 0.1 $ G.text $ "time: " ++ (dec $ time s)
  , G.Translate (-580) (260) $ G.Scale 0.1 0.1 $ G.text $ "FPS: " ++ (dec $ (fromInteger $ toInteger $ frame s) / time s)
  , G.Translate (-580) (240) $ G.Scale 0.1 0.1 $ G.text $ "last hit: " ++ (show $ snd $ head $ hits s)
  , G.Translate (-580) (220) $ G.Scale 0.1 0.1 $ G.text $ "phase: " ++ (show $ phase s)
  ]

drawPhase :: Phase -> State -> Picture
drawPhase DuringRally s = G.Blank
drawPhase (BeforeGame _) s = G.Translate (-580) (100) $ G.Scale 1 1 $ G.text $ (name $ p2 $ s) ++ " VS " ++ (name $ p1 $ s)
drawPhase (BeforeRally _) s = G.Translate (-580) (100) $ G.Scale 1 1 $ G.text $ "prepare for action"
drawPhase (AfterRally _) s = G.Translate (-280) (100) $ G.Scale 1 1 $ G.text $ (show $ snd $ score s) ++ " - " ++ (show $ fst $ score s)
drawPhase (AfterGame _) s = G.Translate (-580) (100) $ G.Scale 1 1 $ G.text $ (name $ winner $ s) ++ " wins!"

dec :: Float -> String
dec x = show $ (fromInteger $ floor $ 100 * x) / 100

center :: Picture -> Picture
center = G.Scale 200 200 . G.Translate 0 (-1)

toFloat :: RealFrac r => Point 2 r -> (Float, Float)
toFloat (Point2 x y) = (realToFrac x, realToFrac y)

drawAt :: RealFrac r => Point 2 r -> Picture -> Picture
drawAt p = uncurry G.Translate (toFloat p)


drawStatic :: Picture
drawStatic = G.Pictures [drawRoom, drawTable, drawNet]

drawRoom :: Picture
drawRoom = G.Color (glossColor cRoom) $ glossify room -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawTable :: Picture
drawTable = G.Color (glossColor cTable) $ glossify table -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawNet :: Picture
drawNet = G.Color (glossColor cNet) $ glossify net -- Line [(0, 0.5), (0, 0.6)]

drawBall :: BallState -> Picture
drawBall s = drawAt (loc s) $ G.Color (glossColor cBall) $ G.circleSolid 0.02

drawPlayer :: Evaluator -> Player -> Bool -> IO Picture
drawPlayer ev p b = do
  pic <- drawArm ev $ arm p
  let pic' = G.Translate (foot p) 0 pic
  case b of True  -> return pic'
            False -> return $ G.Scale (-1) 1 pic'




drawArm :: Evaluator -> Arm -> IO Picture
drawArm ev a = do
  pts <- ev a 
  pol <- evaluateP ev a
  return $ G.Pictures $ glossify pol
                      : drawLinks (armLinks a) pts ++ drawJoints (armJoints a) pts

drawLinks :: [Link] -> [Point 2 Float] -> [Picture]
drawLinks ls ps = zipWith3 drawLink ls (init ps) (tail ps)

drawJoints :: [Joint] -> [Point 2 Float] -> [Picture]
drawJoints js ps = zipWith drawJoint js (tail ps)



drawLink :: Link -> Point 2 Float -> Point 2 Float -> Picture
drawLink l p q = G.Color (glossColor $ lcol l) $ thickLine 0.012 $ map glossify [p, q]
  
drawJoint :: Joint -> Point 2 Float -> Picture
drawJoint j p = G.Color (glossColor $ jcol j) $ drawAt p $ G.circleSolid 0.014
  


glossColor :: Colour Float -> G.Color
glossColor c = 
  let RGB r g b = toSRGB c
  in  G.makeColor r g b 1
