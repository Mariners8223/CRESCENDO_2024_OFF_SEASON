package frc.util.FastGyros

import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

class FastPigeon(canID: Int) : FastGyro {
  private val pigeon: Pigeon2 = Pigeon2(canID)

  private val inputs: FastPigeonInputs = FastPigeonInputs()

  init{
    inputs.canID = canID
    pigeon.setYaw(0.0)
  }

  override fun getAngleDegrees(): Double {
    return  inputs.angle.degrees
  }

  override fun getRotation2d(): Rotation2d {
    return inputs.angle
  }

  override fun update() {
    inputs.angle = pigeon.rotation2d

    inputs.yaw = pigeon.yaw.valueAsDouble
    inputs.pitch = pigeon.pitch.valueAsDouble
    inputs.roll = pigeon.roll.valueAsDouble

    inputs.accelerationX = inputs.angle.cos * pigeon.accelerationX.valueAsDouble - inputs.angle.sin * pigeon.accelerationY.valueAsDouble
    inputs.accelerationY = inputs.angle.sin * pigeon.accelerationX.valueAsDouble + inputs.angle.cos * pigeon.accelerationY.valueAsDouble
  }

  override fun getYaw(): Double {
    return inputs.angle.degrees
  }

  override fun getPitch(): Double {
    TODO("Not yet implemented")
  }

  override fun getRoll(): Double {
    TODO("Not yet implemented")
  }

  override fun getVelocityX(): Double {
    TODO("Not yet implemented")
  }

  override fun getVelocityY(): Double {
    TODO("Not yet implemented")
  }

  override fun getAccelerationX(): Double {
    TODO("Not yet implemented")
  }

  override fun getAccelerationY(): Double {
    TODO("Not yet implemented")
  }

  override fun reset(newPose: Pose2d) {
    TODO("Not yet implemented")
  }

  override fun initSendable(builder: SendableBuilder?) {
    TODO("Not yet implemented")
  }
}

class FastPigeonInputs : LoggableInputs, Cloneable{
  var canID: Int = 0

  var angle: Rotation2d = Rotation2d()
  var rotationOffset: Rotation2d = Rotation2d()

  var velocityX: Double = 0.0
  var velocityY: Double = 0.0
  var accelerationX: Double = 0.0
  var accelerationY: Double = 0.0
  var estimatedPose: Pose2d = Pose2d()

  var prevVelocityX: Double = 0.0
  var prevVelocityY: Double = 0.0
  var prevAccelrationX: Double = 0.0
  var prevAccelrationY: Double = 0.0
  var prevTimeStamp: Long = 0L

  var yaw: Double = 0.0
  var pitch: Double = 0.0
  var roll: Double = 0.0

  override fun toLog(table: LogTable?) {
    TODO("Not yet implemented")
  }

  override fun fromLog(table: LogTable?) {
    TODO("Not yet implemented")
  }

}