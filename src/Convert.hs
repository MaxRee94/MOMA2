module Convert where

import Control.Lens

import qualified Graphics.Gloss as G

import qualified Graphics.Rasterific as R
import Graphics.Rasterific.Texture


import qualified Data.CircularSeq as C
import qualified Data.LSeq as L
import Data.Foldable
import Data.Geometry hiding (init)
import Data.Ext

import Data.Geometry.Point
import Data.Geometry.Polygon
import Data.Geometry.LineSegment
import Data.Geometry.BezierSpline
import Data.Geometry.PlanarSubdivision

import Algorithms.Geometry.PolygonTriangulation.Triangulate

import           GHC.TypeNats










-- conversion to Rasterific geometry types

class GeometryToRasterific a b where
  rasterify :: a -> b

instance Real r => GeometryToRasterific (Point 2 r) (R.Point) where
  rasterify = rasterifyPoint

instance RealFrac r => GeometryToRasterific (Polygon t p r) [R.Primitive] where
  rasterify = rasterifyPolygon

instance RealFrac r => GeometryToRasterific (PolyLine 2 p r) [R.Primitive] where
  rasterify = rasterifyPolyLine

instance Real r => GeometryToRasterific (LineSegment 2 p r) [R.Primitive] where
  rasterify = rasterifySegment

instance (GeometryToRasterific a c, GeometryToRasterific b c) => GeometryToRasterific (Either a b) c where
  rasterify (Left  l) = rasterify l
  rasterify (Right r) = rasterify r









rasterifyPoint :: Real r => Point 2 r -> R.Point
rasterifyPoint (Point2 x y) = R.V2 (realToFrac x) (realToFrac y)

rasterifyPolygon :: (RealFrac) r => Polygon t p r -> [R.Primitive]
rasterifyPolygon p = R.polygon $ map rasterifyPoint $ map _core $ toPoints p

-- Not needed to triangulate? I hope.
{-
rasterifyPolygon p = 
  let t  = triangulate undefined p
      fs = toList $ faces' t
      is = filter (\f -> t ^. dataOf f == Inside) fs
      ts = map _core $ map (flip rawFaceBoundary t) is
  in concat $ map rasterifyConvexPolygon ts
-}

rasterifyPolyLine :: (RealFrac) r => PolyLine 2 p r -> [R.Primitive]
rasterifyPolyLine (PolyLine s) = R.polyline $ rasterifyLSeq s
  
rasterifyCSeq :: Real r => C.CSeq (Point 2 r :+ p) -> [R.Point]
rasterifyCSeq = map rasterifyPoint . map _core . toList

rasterifyLSeq :: Real r => L.LSeq 2 (Point 2 r :+ p) -> [R.Point]
rasterifyLSeq = map rasterifyPoint . map _core . toList

rasterifySegment :: Real r => LineSegment 2 p r -> [R.Primitive]
rasterifySegment s = R.line (rasterify $ s ^. start ^. core) (rasterify $ s ^. end ^. core)












class Glossifiable a b where
  glossify :: a -> b

instance Real r => Glossifiable (Point 2 r) G.Point where
  glossify = glossifyPoint

instance RealFrac r => Glossifiable (Polygon t p r) G.Picture where
  glossify = glossifyPolygon

instance RealFrac r => Glossifiable (PolyLine 2 p r) G.Picture where
  glossify = glossifyPolyLine

instance Real r => Glossifiable (LineSegment 2 p r) G.Picture where
  glossify = glossifySegment

instance (Glossifiable a c, Glossifiable b c) => Glossifiable (Either a b) c where
  glossify (Left  l) = glossify l
  glossify (Right r) = glossify r




glossifyPoint :: Real r => Point 2 r -> G.Point
glossifyPoint (Point2 x y) = (realToFrac x, realToFrac y)

glossifyPolygon :: (RealFrac) r => Polygon t p r -> G.Picture
glossifyPolygon p = G.Polygon $ map glossifyPoint $ map _core $ toPoints p

glossifyConvexPolygon :: RealFrac r => Polygon t p r -> G.Picture
glossifyConvexPolygon p = G.Polygon $ map glossifyPoint $ map _core $ toPoints p

glossifyPolyLine :: (RealFrac) r => PolyLine 2 p r -> G.Picture
glossifyPolyLine (PolyLine s) =  G.Line $ glossifyLSeq s
  
glossifyCSeq :: Real r => C.CSeq (Point 2 r :+ p) -> [G.Point]
glossifyCSeq = map glossifyPoint . map _core . toList

glossifyLSeq :: Real r => L.LSeq 2 (Point 2 r :+ p) -> [G.Point]
glossifyLSeq = map glossifyPoint . map _core . toList

glossifySegment :: Real r => LineSegment 2 p r -> G.Picture
glossifySegment s = let (p :+ _, q :+ _) = orderedEndPoints s 
                    in G.Line $ map glossifyPoint [p, q]









thickLine :: Float -> G.Path -> G.Picture
thickLine w ps = G.Pictures $ map v ps ++ map e (zip (init ps) (tail ps))
  where
    v :: G.Point -> G.Picture
    v (x, y) = G.translate x y $ G.circleSolid $ w/2
    e :: (G.Point, G.Point) -> G.Picture
    e ((px, py), (qx, qy)) = 
      let l  = sqrt $ (qx - px) ** 2 + (qy - py) ** 2
          dx = (qx - px) / l 
          dy = (qy - py) / l 
      in G.Polygon [ (px + w/2 * dy, py - w/2 * dx)
                   , (px - w/2 * dy, py + w/2 * dx)
                   , (qx - w/2 * dy, qy + w/2 * dx)
                   , (qx + w/2 * dy, qy - w/2 * dx)
                   ]
