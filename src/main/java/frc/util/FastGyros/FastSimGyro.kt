package frc.util.FastGyros

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
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

class FastSimGyro(private val twistSupplier: Supplier<Twist2d>) : FastGyro{
  private val lock : ReentrantLock = ReentrantLock()
  private val inputs: SimGyroInputs = SimGyroInputs()


  override fun getAngleDegrees(): Double {
    try {
      lock.lock()
      return inputs.angle.degrees
    }
    finally {
      lock.unlock()
    }
  }

  override fun getRotation2d(): Rotation2d {
    try {
      lock.lock()
      return inputs.angle
    }
    finally {
      lock.unlock()
    }
  }


  override fun update() : Unit {
    try{
      lock.lock()
      // SmartDashboard.putNumber("SimGyro dta", twistSupplier.get().dtheta)
      inputs.angle = Rotation2d.fromRadians(twistSupplier.get().dtheta + inputs.angle.radians)
      // Logger.processInputs("SimGyro", inputs)
    }
    finally{
      lock.unlock()
    }
  }

  override fun getYaw(): Double {
    try {
      lock.lock()
      return -inputs.angle.degrees
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
    return 0.0
  }

  override fun getVelocityY(): Double {
    return 0.0
  }

  override fun getAccelerationX(): Double {
    return 0.0
  }

  override fun getAccelerationY(): Double {
    return 0.0
  }

  override fun reset(newPose: Pose2d) {
    inputs.angle = newPose.rotation
  }

  override fun initSendable(builder: SendableBuilder?) {
  }
}



class SimGyroInputs : LoggableInputs, Cloneable{
  public var angle: Rotation2d = Rotation2d()

  override fun toLog(table: LogTable) {
    table.put("angle", angle)
  }

  override fun fromLog(table: LogTable) {
    angle = table.get("angle", angle)[0]
  }

}