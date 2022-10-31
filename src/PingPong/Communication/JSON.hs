{-# LANGUAGE OverloadedStrings #-}

module PingPong.Communication.JSON where

import PingPong.Model
import PingPong.Simulation.Collision (Snapshot)

import Data.Colour
import Data.Colour.SRGB
import Data.Geometry
import Data.Aeson
import Data.Ext

import GHC.Generics

import Data.Aeson
import Data.Aeson.Types

import Control.Lens hiding ((.=))


-- For now, just generate default instances. To be modified as required.

-- instance ToJSON   Joint     where toEncoding = genericToEncoding defaultOptions
-- instance ToJSON   Link      where toEncoding = genericToEncoding defaultOptions
-- instance ToJSON   Arm       where toEncoding = genericToEncoding defaultOptions
-- instance ToJSON   BallState where toEncoding = genericToEncoding defaultOptions
instance FromJSON BallState
-- instance ToJSON   Item      where toEncoding = genericToEncoding defaultOptions
instance FromJSON Item
instance ToJSON   Owner     where toEncoding = genericToEncoding defaultOptions
instance FromJSON Owner

-- Need to also have instances for other types we use.

instance FromJSON (Colour Float) where
    parseJSON = withObject "Colour" $ \v -> parseColour
        <$> v .: "colour"

-- The code that was there did not parse hex-colors correctly somehow, 
-- o I changed it to some library function, to be changed for error handling
parseColour :: String -> Colour Float
parseColour s = sRGB24read s
-- parseColour ['#', a, b, c, d, e, f] = sRGB24 (read [a, b]) (read [c, d]) (read [e, f])
-- parseColour s = error $ "parseColour: \"" ++ s ++ "\" is not in the correct format" 

printColour :: Colour Float -> String
printColour c = sRGB24show c

instance ToJSON (Colour Float) where
    -- this generates a Value
    toJSON col =
        object ["colour" .= printColour col]

    -- this encodes directly to a bytestring Builder
    toEncoding col =
        pairs ("colour" .= printColour col)

-- instance Show a => ToJSON (LineSegment 2 a Float) where
--     toJSON seg = object ["segment" .= show seg]

-- JSON for Link
instance FromJSON Link where
    parseJSON = withObject "Link" $ \v -> parseLink
        <$> v .: "colour"
        <*> v .: "length"

parseLink :: String -> Float -> Link
parseLink col len = Link (parseColour col) len

instance ToJSON Link where
    toJSON link =
        object ["length" .= llen link, "colour" .= printColour (lcol link)]

    toEncoding link =
        pairs ("length" .= llen link <> "colour" .= printColour (lcol link))

-- JSON for Joint
instance FromJSON Joint where
    parseJSON = withObject "Joint" $ \v -> parseJoint
        <$> v .: "colour"
        <*> v .: "angle"
        <*> v .: "velocity"

parseJoint :: String -> Float -> Float -> Joint
parseJoint col ang vel = Joint (parseColour col) ang vel

instance ToJSON Joint where
    toJSON joint =
        object ["angle" .= jang joint, "velocity" .= jvel joint, "colour" .= printColour (jcol joint)]

    toEncoding joint =
        pairs ("angle" .= jang joint <> "velocity" .= jvel joint <> "colour" .= printColour (jcol joint))

-- JSON for Arm
instance FromJSON Arm where
    parseJSON = withObject "Arm" $ \v -> parseArm
        <$> v .: "links"
        <*> v .: "joints"

parseArm :: [Link] -> [Joint] -> Arm
parseArm ls js = makeArm (zip (Prelude.init ls) js) (End $ last ls)

instance ToJSON Arm where
    toJSON arm =
        object ["links" .= armLinks arm, "joints" .= armJoints arm]

    toEncoding arm =
        pairs ("links" .= armLinks arm <> "joints" .= armJoints arm)

-- JSON for BallState
instance ToJSON BallState where
    toJSON state =
        object ["location" .= loc state, "direction" .= dir state]
    
    toEncoding state =
        pairs ("location" .= loc state <> "direction" .= dir state)

item_id :: Item -> Int
item_id Air = 0
item_id (Bat Self) = 1
item_id (Bat Opponent) = 2
item_id (Table Self) = 3
item_id (Table Opponent) = 4
item_id Net = 5
item_id (Other x) = 100 + x

-- JSON for Item
instance ToJSON Item where
    toJSON item = toJSON $ item_id item

-- JSON for Snapshot
encodeSnapshot :: Snapshot -> Value
encodeSnapshot (time, pnt, seg) =
    object ["time" .= time, "ball" .= pnt, "segment" .= seg]

-- JSON for Segment
instance ToJSON (LineSegment 2 () Float) where
    toJSON seg = let startp = seg ^. start . core
                     endp = seg ^. end . core
                 in object ["start" .= startp, "end" .= endp]

    toEncoding seg = let startp = seg ^. start . core
                         endp = seg ^. end . core
                 in pairs ("start" .= startp <> "end" .= endp)

instance FromJSON (LineSegment 2 () Float) where
    parseJSON = withObject "Seg" $ \v -> parseSeg
        <$> v .: "start"
        <*> v .: "end"

parseSeg :: Pnt -> Pnt -> Seg
parseSeg p q = OpenLineSegment (p :+ ()) (q :+ ())

-- Direct parser for messages
parseSetup s = flip parseEither s $ \obj -> do
                    name <- obj .: "name" :: Parser String
                    arm <- obj .: "arm" :: Parser Arm
                    return (name, arm)

parseControl c = flip parseEither c $ \obj -> do
                     control <- obj .: "control" :: Parser Control
                     return control

parseCollisionDetection c = flip parseMaybe c $ \obj -> do
                                    collision <- obj .: "collision" :: Parser (Second)
                                    return collision

parseCollisionHandling c = flip parseEither c $ \obj -> do
                                    locball <- obj .: "ball" :: Parser Pnt
                                    velball <- obj .: "velocity" :: Parser Vec
                                    return (locball, velball)

parseControlArm c = flip parseEither c $ \obj -> do
                            arm <- obj .: "arm" :: Parser Arm
                            return arm

parseEvaluateArm e = flip parseEither e $ \obj -> do
                            locations <- obj .: "locations" :: Parser [Pnt]
                            return locations

parseInverse i = flip parseEither i $ \obj -> do
                            angles <- obj .: "angles" :: Parser (Maybe [Radian])
                            return angles

parseDance d = parseControl d
parsePlan x = parseControl x
parseAction x = parseControl x