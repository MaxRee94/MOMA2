module PingPong.Draw.Rasterific where

import PingPong.Model
import PingPong.Draw

import Convert
import PingPong.Simulation.ForwardKinematics

import Codec.Picture( Image, PixelRGBA8( .. ), writePng )
import qualified Graphics.Rasterific as R
import Graphics.Rasterific.Texture

import Control.Lens

import Data.Geometry hiding (init)
import Data.Geometry.Point
import Data.Geometry.Polygon
import Data.Geometry.LineSegment
import Data.Geometry.PlanarSubdivision

import Algorithms.Geometry.PolygonTriangulation.Triangulate

import Data.Ext
import Data.Foldable
import qualified Data.CircularSeq as C
import qualified Data.LSeq as L

import Graphics.Text.TrueType( Font, loadFontFile )


import           GHC.TypeNats

type Color = PixelRGBA8 -- use proper colour
type Drawing = R.Drawing PixelRGBA8 ()


linkWidth, jointRadius, ballRadius :: Float
linkWidth = 2
jointRadius = 2
ballRadius = 4
wRoom = 1

renderState :: Evaluator -> Font -> State -> IO (Image PixelRGBA8)
renderState ev font state = do
  drawing <- drawState ev font state
  let white   = PixelRGBA8 255 255 255 255
      image   = R.renderDrawing 1920 1080 white drawing
  return $ image

drawState :: Evaluator -> Font -> State -> IO Drawing
drawState ev font m = do
  drawPlayer1 <- drawPlayer ev (p1 m) True
  drawPlayer2 <- drawPlayer ev (p2 m) False
  return $ do drawStatic
              drawPlayer1
              drawPlayer2
              drawBall $ ball m
              drawInfo font (name $ p1 m) (name $ p2 m)
              drawScore font (fst $ score m) (snd $ score m)
              drawPhase font (phase m) m
--                 withColor (convertColor cRoom) $ R.fill $ R.rectangle (R.V2 100 100) 200 100

--center :: IsTransformable a => a -> a
center :: (IsTransformable g, Num (NumType g), Dimension g ~ 2) => g -> g
-- center = id
-- center = scaleUniformlyBy 200 . translateBy (Vector2 0 (-1))
center = scaleUniformlyBy 320 
       . scaleBy (Vector2 1 (-1))
       . translateBy (Vector2 3 (-3))


drawStatic :: Drawing
drawStatic = do drawRoom
                drawTable
                drawNet

drawPhase :: Font -> Phase -> State -> Drawing
drawPhase font DuringRally s = return ()
drawPhase font (BeforeGame _) s = drawText font (convertColor cTable) 64 500 300 $ (name $ p2 $ s) ++ " VS " ++ (name $ p1 $ s)
drawPhase font (BeforeRally _) s = drawText font (convertColor cTable) 64 500 300 $ "prepare for action"
drawPhase font (AfterRally _) s = drawText font (convertColor cTable) 64 800 300 $ (show $ snd $ score s) ++ " - " ++ (show $ fst $ score s)
drawPhase font (AfterGame _) s = drawText font (convertColor cTable) 64 500 300 $ (name $ winner $ s) ++ " wins!"


drawInfo :: Font -> String -> String -> Drawing
drawInfo font n1 n2 = do
  drawText font (convertColor cTable) 36 300 100 n2
  drawText font (convertColor cTable) 48 900 100 "VS"
  drawText font (convertColor cTable) 36 1300 100 n1


drawScore :: Font -> Int -> Int -> Drawing
drawScore font n1 n2 = do 
  drawText font (convertColor cTable) 64 400 200 $ show n2
  drawText font (convertColor cTable) 64 1400 200 $ show n1

drawText :: Font -> Color -> Float -> Float -> Float -> String -> Drawing
drawText font color size x y text = withColor color $ R.printTextAt font (R.PointSize size) (R.V2 x y) text

drawRoom :: Drawing
drawRoom = withColor (convertColor cRoom)
         $ R.fill
         $ rasterifyPolygon $ center room -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawTable :: Drawing
drawTable = do withColor (convertColor cTable) $ R.fill $ rasterifyPolygon $ center table -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]
--               withColor (convertColor cTable) $ R.stroke wRoom R.JoinRound (R.CapRound, R.CapRound)  $ rasterifyPolygon $ center table -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawNet :: Drawing
drawNet = withColor (convertColor cNet)
        $ R.stroke wRoom R.JoinRound (R.CapRound, R.CapRound)
        $ rasterifySegment $ center net -- Line [(0, 0.5), (0, 0.6)]

drawBall :: BallState -> Drawing
drawBall s = withColor (convertColor cBall) $ R.fill $ R.circle (rasterifyPoint $ center $ loc s) ballRadius

drawPlayer :: Evaluator -> Player -> Bool -> IO Drawing
drawPlayer ev p b = drawArm ev (arm p) (foot p) b


drawArm :: Evaluator -> Arm -> Float -> Bool -> IO Drawing
drawArm ev a d b = do
  pts <- ev a
  return $ do
    let ps = map (center . playerTransform d b) pts
    drawLinks (armLinks a) ps
    drawJoints (armJoints a) ps

drawLinks :: [Link] -> [Point 2 Float] -> Drawing
drawLinks ls ps = sequence_ $ zipWith3 drawLink ls (init ps) (tail ps)

drawJoints :: [Joint] -> [Point 2 Float] -> Drawing
drawJoints js ps = sequence_ $ zipWith drawJoint js ps


playerTransform :: (IsTransformable g, NumType g ~ Float, Dimension g ~ 2) => Float -> Bool -> g -> g
--playerTransform :: Float -> Bool -> Point 2 Float -> Point 2 Float
--playerTransform :: (IsTransformable g, Num (NumType g), Dimension g ~ 2) => Float -> Bool -> g -> g
playerTransform d True = translateBy $ Vector2 d 0
playerTransform d False = scaleBy (Vector2 (-1) 1) . translateBy (Vector2 d 0)


drawLink :: Link -> Point 2 Float -> Point 2 Float -> Drawing
drawLink l p q = withColor (convertColor $ lcol l)
               $ R.stroke linkWidth R.JoinRound (R.CapRound, R.CapRound) 
               $ R.line (rasterify p) (rasterify q)

drawJoint :: Joint -> Point 2 Float -> Drawing
drawJoint j p = withColor (convertColor $ jcol j)
              $ R.fill
              $ R.circle (rasterifyPoint p) jointRadius



withColor :: Color -> Drawing -> Drawing
withColor c = R.withTexture $ uniformTexture c

-- rasterifySegment




