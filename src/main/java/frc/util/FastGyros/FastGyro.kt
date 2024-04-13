package frc.util.FastGyros

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d

interface FastGyro {
  public fun getAngle(): Double
  public fun getRotation2d(): Rotation2d
  public fun getOriginalAngle(): Double
  public fun reset(newPose: Pose2d = Pose2d()): Unit
}