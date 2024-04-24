package frc.util.FastGyros

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import java.util.concurrent.locks.ReentrantLock

interface FastGyro : Sendable {
  fun getAngleDegrees(): Double
  fun getRotation2d(): Rotation2d
  fun update(): Unit
  fun getYaw(): Double
  fun getPitch(): Double
  fun getRoll(): Double
  fun getVelocityX(): Double
  fun getVelocityY(): Double
  fun getAccelerationX(): Double
  fun getAccelerationY(): Double
  fun reset(newPose: Pose2d = Pose2d()): Unit
  override fun initSendable(builder: SendableBuilder?): Unit
}