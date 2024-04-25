package frc.util.FastGyros

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs
import java.util.concurrent.locks.ReentrantLock
import java.util.function.Supplier
import kotlin.math.cos
import kotlin.math.sin

class FastSimGyro(private val twistSupplier: Supplier<Twist2d>, private val chassisSpeedsSupplier: Supplier<ChassisSpeeds>) : FastGyro{
  private val lock : ReentrantLock = ReentrantLock()
  private var angle: Rotation2d = Rotation2d()

  private var velocityX: Double = 0.0
  private var velocityY: Double = 0.0

  private var displacmentX: Double = 0.0
  private var displacmentY: Double = 0.0

  private var accelerationX: Double = 0.0
  private var accelerationY: Double = 0.0

  private var prevVelocityX: Double = 0.0
  private var prevVelocityY: Double = 0.0
  private var prevTimestamp: Long = 0L


  override fun getAngleDegrees(): Double {
    try {
      lock.lock()
      return angle.degrees
    }
    finally {
      lock.unlock()
    }
  }

  override fun getRotation2d(): Rotation2d {
    try {
      lock.lock()
      return angle
    }
    finally {
      lock.unlock()
    }
  }


  override fun update() : Unit {
    try{
      lock.lock()
      val twist = twistSupplier.get()
      angle = Rotation2d.fromRadians(twist.dtheta + angle.radians)
      Logger.recordOutput("Gyro/angle", angle)

      displacmentX += twist.dx * cos(angle.radians) + twist.dy * sin(angle.radians)
      displacmentY += twist.dx * sin(angle.radians) + twist.dy * cos(angle.radians)

      Logger.recordOutput("Gyro/EstimatedPose", Pose2d(displacmentX, displacmentY, angle))

      velocityX = chassisSpeedsSupplier.get().vxMetersPerSecond
      velocityY = chassisSpeedsSupplier.get().vyMetersPerSecond

      accelerationX = (velocityX - prevVelocityX) / (Logger.getTimestamp() - prevTimestamp)
      accelerationY = (velocityY - prevVelocityY) / (Logger.getTimestamp() - prevTimestamp)

      prevVelocityX = velocityX
      prevVelocityY = velocityY

      prevTimestamp = Logger.getTimestamp()

      Logger.recordOutput("Gyro/velocityX", velocityX)
      Logger.recordOutput("Gyro/velocityY", velocityY)
      Logger.recordOutput("Gyro/accelerationX", accelerationX)
      Logger.recordOutput("Gyro/accelerationY", accelerationY)
    }
    finally{
      lock.unlock()
    }
  }

  override fun getYaw(): Double {
    try {
      lock.lock()
      return -angle.degrees
    }
    finally {
      lock.unlock()
    }
  }

  override fun getPitch(): Double {
    return 0.0
  }

  override fun getRoll(): Double {
    return 0.0
  }

  override fun getVelocityX(): Double {
    try {
      lock.lock()
      return velocityX
    }
    finally {
      lock.unlock()
    }
  }

  override fun getVelocityY(): Double {
    try {
      lock.lock()
      return velocityY
    }
    finally {
      lock.unlock()
    }
  }

  override fun getAccelerationX(): Double {
    try {
      lock.lock()
      return accelerationX
    }
    finally {
      lock.unlock()
    }
  }

  override fun getAccelerationY(): Double {
    try {
      lock.lock()
      return accelerationY
    }
    finally {
      lock.unlock()
    }
  }

  override fun reset(newPose: Pose2d) {
    try {
      lock.lock()
      angle = newPose.rotation
      displacmentX = newPose.translation.x
      displacmentY = newPose.translation.y
    }
    finally {
      lock.unlock()
    }
  }

  override fun initSendable(builder: SendableBuilder?) {
    builder?.setSmartDashboardType("Gyro")
    builder?.addDoubleProperty("Value", this::getAngleDegrees) { value: Double ->
      reset(
        Pose2d(
          0.0,
          0.0,
          Rotation2d.fromDegrees(value)
        )
      )
    }
  }
}