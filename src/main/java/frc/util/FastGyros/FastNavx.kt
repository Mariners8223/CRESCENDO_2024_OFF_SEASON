@file:JvmName("MessageHelper") 
package frc.util.FastGyros

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.AutoLog




class FastNavx : FastGyro{

  @AutoLog open class NavxInputs {
    protected var angle: Double = 0.0
    protected var velocityX: Double = 0.0
    protected var velocityY: Double = 0.0
    protected var accelerationX: Double = 0.0
    protected var accelerationY: Double = 0.0
  }

  private val navx: AHRS = AHRS()
  private var rotationOffset: Rotation2d = Rotation2d()

  private var displacmentX: Double = 0.0
  private var displacmentY: Double = 0.0

  private var prevVelocityX: Double = 0.0
  private var prevVelocityY: Double = 0.0
  private var prevTimeStamp: Double = 0.0

//  private val inputs: NavxInputsAutoLogged = NavxInputsAutoLogged()


  override fun reset(newPose: Pose2d): Unit {
    navx.reset()
    displacmentX = newPose.translation.x
    displacmentY = newPose.translation.y
    rotationOffset = newPose.rotation
  }

  override fun getAngle(): Double {
    return navx.angle
  }

  override fun getRotation2d(): Rotation2d {
    return navx.rotation2d
  }

  override fun getOriginalAngle(): Double {
    return navx.angle
  }

  fun update(): Unit {
    var currentVelocityX = Math.sin(Units.degreesToRadians(0.0)) * navx.velocityX
    var currentVelocityY = Math.cos(Units.degreesToRadians(0.0)) * navx.velocityY
  }
}