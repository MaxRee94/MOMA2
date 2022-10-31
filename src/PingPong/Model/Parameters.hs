module PingPong.Model.Parameters where


-- | Maximum angular joint velocity in radians per second.
maxSpeed :: Float -- RadianPerSecond
maxSpeed = 2

-- | Maximum joint acceleration in radians per square second.
maxAcceleration :: Float -- RadianPerSquareSecond
maxAcceleration = 5

-- | Minimum number of joints in a robot arm.
minNumberOfJoints :: Int
minNumberOfJoints = 2

-- | Maximum number of joints in a robot arm.
maxNumberOfJoints :: Int
maxNumberOfJoints = 5

-- | Minimum length of a link in a robot arm.
minLinkLength :: Float -- Meter
minLinkLength = 0.1

-- | Maximum length of a link in a robot arm.
maxLinkLength :: Float -- Meter
maxLinkLength = 0.5

-- | Length of the last link in a robot arm (the bat).
batLength :: Float -- Meter
batLength = 0.1
