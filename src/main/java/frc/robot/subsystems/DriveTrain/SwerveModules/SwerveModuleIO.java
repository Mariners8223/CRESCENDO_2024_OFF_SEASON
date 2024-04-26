package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public SwerveModuleState currentState = new SwerveModuleState();

    public double steerVelocityRadPerSec = 0.0;
    public double drivePositionMeters = 0.0;

    public double steerMotorAppliedVoltage = 0.0;
    public double driveMotorAppliedVoltage = 0.0;
  }

  /**
   * Updates the inputs of the module
   */
  public default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {};

  /**
   * sets the voltage for the drive motor
   * @param voltage the voltage to set the drive motor to
   */
  public default void setDriveMotorVoltage(double voltage) {};

  /**
   * sets the voltage for the steer motor
   * @param voltage the voltage to set the steer motor to
   */
  public default void setSteerMotorVoltage(double voltage) {};

  /**
   * sets the idle mode of the module
   * @param isBrakeMode true for brake mode, false for coast mode
   */
  public default void setIdleMode(boolean isBrakeMode) {};

  /**
   * resets the drive encoder
   */
  public default void resetDriveEncoder() {};

}
