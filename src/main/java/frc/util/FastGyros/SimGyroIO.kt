package frc.util.FastGyros

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.Constants
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule
import org.littletonrobotics.junction.Logger
import java.util.concurrent.locks.ReentrantLock
import java.util.function.Supplier

class SimGyroIO(private val twistSupplier: Supplier<Twist2d>, private val chassisSpeedsSupplier: Supplier<ChassisSpeeds>) : GyroIO{
  private val lock : ReentrantLock = ReentrantLock()
  private var angle: Rotation2d = Rotation2d()

  private var velocityX: Double = 0.0
  private var velocityY: Double = 0.0

  private var displacementX: Double = 0.0
  private var displacementY: Double = 0.0

  private var accelerationX: Double = 0.0
  private var accelerationY: Double = 0.0

  private var prevVelocityX: Double = 0.0
  private var prevVelocityY: Double = 0.0

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


  override fun update() {
    try{
      lock.lock()
      val twist = twistSupplier.get()
      angle = Rotation2d.fromRadians(twist.dtheta + angle.radians)
      Logger.recordOutput("Gyro/angle", angle)

      displacementX += twist.dx * angle.cos - twist.dy * angle.sin
      displacementY += twist.dy * angle.cos + twist.dx * angle.sin

      Logger.recordOutput("Gyro/EstimatedPose", Pose2d(displacementX, displacementY, angle))

      val chassisSpeeds = chassisSpeedsSupplier.get()

//      velocityX = chassisSpeeds.vxMetersPerSecond * angle.cos - chassisSpeeds.vyMetersPerSecond * angle.sin
//      velocityY = chassisSpeeds.vyMetersPerSecond * angle.cos + chassisSpeeds.vxMetersPerSecond * angle.sin
      velocityX = chassisSpeeds.vxMetersPerSecond
      velocityY = chassisSpeeds.vyMetersPerSecond

      accelerationX = (velocityX - prevVelocityX) / (1 / SwerveModule.SwerveModuleConstants.moduleThreadHz)
      accelerationY = (velocityY - prevVelocityY) / (1 / SwerveModule.SwerveModuleConstants.moduleThreadHz)

      Logger.recordOutput("Gyro/PrevVelocityX", prevVelocityX)
      Logger.recordOutput("Gyro/PrevVelocityY", prevVelocityY)

      prevVelocityX = velocityX
      prevVelocityY = velocityY

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
      displacementX = newPose.translation.x
      displacementY = newPose.translation.y
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