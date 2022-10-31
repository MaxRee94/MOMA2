module PingPong.Model where

import PingPong.Model.Parameters

import Data.Geometry hiding (init)
--import Data.Geometry.Matrix
--import Data.Geometry.Transformation
import Data.Geometry.PolyLine
import Data.Geometry.Polygon

import Data.Ext
import Data.List hiding (filter, map, null)
import Data.Either
import Control.Lens

import Data.Colour

import GHC.Generics


-- * Model of all components of the game.

-- | Type synonyms for readability.
type Radian                = Float
type Second                = Float
type RadianPerSecond       = Float
type RadianPerSquareSecond = Float
type Meter                 = Float

type Pnt = Point 2 Float
type Vec = Vector 2 Float
type Seg = LineSegment 2 () Float

data Joint = Joint
  { jcol :: Colour Float    -- ^ Colour used for drawing the joint.
  , jang :: Radian          -- ^ Current angle of the joint.
  , jvel :: RadianPerSecond -- ^ Angular velocity of the joint.
  } deriving (Show, Eq, Generic)

data Link = Link
  { lcol :: Colour Float -- ^ Colour used for drawing the link.
  , llen :: Meter        -- ^ Length of the link.
  } deriving (Show, Eq, Generic)

type Element = Either Link Joint

-- | A robotic arm is a combination of multiple links and joints.
--   The first and last elements should be links; the last link is the bat.
data Arm = Extend Link Joint Arm | End Link
  deriving (Show, Eq, Generic)


(-*) :: Link -> Joint -> (Link, Joint)
l -* j = (l, j)
(*-) :: (Link, Joint) -> Arm -> Arm
(l, j) *- a = Extend l j a

infixr 9 -*
infixr 8 *-

link :: Colour Float -> Float -> Link
link = Link

bat :: Colour Float -> Arm
bat c = End $ link c 0.1

joint :: Colour Float -> Radian -> Joint
joint c r = Joint c r 0


armElements :: Arm -> [Element]
armElements (End l)        = [Left l]
armElements (Extend l j a) = Left l : Right j : armElements a

armLinks :: Arm -> [Link]
armLinks = lefts . armElements

armJoints :: Arm -> [Joint]
armJoints = rights . armElements

-- | Check if a given arm respects the restrictions.
validateArm :: Arm -> Bool
validateArm arm | length (armJoints arm) < minNumberOfJoints = False
                | length (armJoints arm) > maxNumberOfJoints = False
                | any ((< minLinkLength) . llen) $ armLinks arm = False
                | any ((> maxLinkLength) . llen) $ armLinks arm = False                
                | llen (last $ armLinks arm) /= batLength = False
                | otherwise = True

asLink :: Element -> Link
asLink (Left e) = e

asJoint :: Element -> Joint
asJoint (Right e) = e

makeArm :: [(Link, Joint)] -> Arm -> Arm
-- makeArm es arm = makeArm (take 2 es) $ (asLink $ es!!1) -* (asJoint $ es!!2) *- arm
makeArm [lj] bat = lj *- bat
makeArm (lj:ljs) bat = lj *- (makeArm ljs bat)


cap :: Float -> Float -> Float
cap x m | x < -m    = -m
        | x >  m    =  m
        | otherwise =  x

capSpeed :: RadianPerSecond -> RadianPerSecond
capSpeed s = cap s maxSpeed

capAcceleration :: RadianPerSquareSecond -> RadianPerSquareSecond
capAcceleration a = cap a maxAcceleration


-- | A Hit stores a moment in time when the ball hit something.
--   This contains both the time of the hit and what got hit.
type Hit = (Second, Item)

-- | A combination of location and velocity of the ball.
data BallState = BallState 
  { loc :: Pnt -- ^ Position of the ball.
  , dir :: Vec -- ^ Velocity of the ball.
  } deriving (Show, Eq, Generic)

-- | Specifies a single player or character that can play table tennis.
data Player = Player 
  { name      :: String   -- ^ Human-readable player name.
  , arm       :: Arm      -- ^ Description of the player arm.
  , initArm   :: Arm      -- ^ Initial state of the arm.
  , foot      :: Float    -- ^ Distance of the base of the arm from the origin.
    -- | This method is called once at the start of the game.
    --   Can for instance be used for setting up infrastructure for communication.
  , prepare   :: IO ()
    -- | This method is called once at the end of the game.
    -- | Provides the action that the robotic arm should perform given
    -- the current state of the world and its own position in space.
  , action    :: Second -> Hit -> BallState -> Arm -> IO Control
    -- | Makes the arm stretch before the start of the game.
  , stretch   :: Second -> Arm -> IO Control
    -- | Makes the robotic arm do a victory dance when it has won the match.
  , dance     :: Second -> Arm -> IO Control


  }

-- | A vector of control values for the joint actuators in radians per square second.
type Control = [RadianPerSquareSecond]  

-- | A vector of angular joint velocities in radians per second.
type Motion = [RadianPerSecond]

-- * Simulation Data

-- | Data type describing possible things the ball can hit.
data Item 
  = Air         -- ^ The ball is not in collision.
  | Bat Owner   -- ^ The ball is in collision with one of the player's bats.
  | Table Owner -- ^ The ball is in collision with the table last on the specified side.
  | Net         -- ^ The ball is in collision with the net.
  -- | Specifies that the ball hit any other surface denoted by an identifier.
  -- This will be the walls of the room, the ceiling or the floor, the underside
  -- of the table, etc. depending on the location of the ball.
  | Other Int
  deriving (Show, Eq, Ord, Generic)

-- | Data type to indicate a player relative to the current player.
data Owner = Self | Opponent
  deriving (Show, Eq, Ord, Generic)

-- | Describes the current situation of the field to determine arm behaviour.
data Phase
    -- | The situation when setting up and initialising the robotic arms.
  = BeforeGame Float
    -- | The situation before the players will face off against each other.
    -- This is used to attempt to revert the players back to 0.
  | BeforeRally Float
    -- | The situation where two players are facing off against each other.
    -- This is the main match when they are playing.
  | DuringRally
    -- | The situation after a set has been played and someone scored a point.
    -- This allows for victory dances of each robotic arm.
  | AfterRally Float
    -- | The situation after all sets have been played and a definitive winner
    -- has been chosen! Now the robotic arms can be cleaned up and terminate
    -- removing all remaining data from both players.
  | AfterGame Float
    -- | State to indicate the process should be terminated.
  | GameOver    
  deriving (Eq, Show, Generic)

-- | Internal data structure for the current game state.
data State = State 
    -- | The current phase or progress within a game.
  { phase  :: Phase
    -- | The time elapsed since the start of the game.
  , time   :: Second
    -- | The number of frames since the start of the game.
  , frame  ::  Int
    -- | The number of points scored by each player.
  , score  :: (Int, Int)
    -- | The current ball state.
  , ball   ::  BallState
    -- | The times and types of the all ball collisions.
  , hits   :: [Hit]
    -- | The left player.
  , p1     ::  Player
    -- | The right player.
  , p2     ::  Player
    -- | The current control vector of the left player.
  , c1     ::  Control
    -- | The current control vector of the right player.
  , c2     ::  Control
  }


-- | Geometry of the room.
room :: SimplePolygon () Float
room = Data.Geometry.Polygon.fromPoints $ map (:+ ()) [Point2 (-3) 0, Point2 3 0, Point2 3 6, Point2 (-3) 6]

-- | Geometry of the table.
table :: SimplePolygon () Float
table = Data.Geometry.Polygon.fromPoints $ map (:+ ()) [Point2 1 0.5, Point2 0 0.5, Point2 (-1) 0.5, Point2 (-1) 0.4, Point2 1 0.4]

-- | Geometry of the net.
net :: LineSegment 2 () Float
net = ClosedLineSegment (Point2 0 0.5 :+ ()) (Point2 0 0.6 :+ ())











-- | Change state from the perspective of p1 to the perspective of p2.
flipState :: State -> State
flipState st = st { ball = flipBall $ ball st
                  , hits = map flipHit $ hits st
                  , p1 = p2 st
                  , p2 = p1 st
                  , c1 = flipControl $ c2 st
                  , c2 = flipControl $ c1 st
                  }

flipBall :: BallState -> BallState
flipBall st = st { loc = transformBy reflectionH $ loc st
                 , dir = transformBy reflectionH $ dir st
                 }

flipControl :: Control -> Control
flipControl = map negate

flipHit :: (Float, Item) -> (Float, Item)
flipHit (f, i) = (f, flipItem i)

flipItem :: Item -> Item
flipItem (Bat   o) = Bat   $ flipOwner o
flipItem (Table o) = Table $ flipOwner o
flipItem o         = o

flipOwner :: Owner -> Owner
flipOwner Self = Opponent
flipOwner Opponent = Self

defState :: State
defState = State
  { phase = BeforeGame 2
  , time  = 0
  , frame = 0
  , score = (0, 0)
  , ball  = BallState (Point2 0 1) (Vector2 1 0)
  , hits  = [(0, Bat Opponent)]
  , p1    = undefined
  , p2    = undefined
  , c1    = undefined
  , c2    = undefined
  }

winner :: State -> Player
winner s | fst (score s) > snd (score s) = p1 s
         | otherwise = p2 s

