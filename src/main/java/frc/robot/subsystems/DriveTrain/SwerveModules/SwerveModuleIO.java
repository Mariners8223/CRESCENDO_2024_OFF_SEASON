package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public SwerveModuleState currentState = new SwerveModuleState();
    public SwerveModuleState targetState = new SwerveModuleState();

    public boolean isAtTargetPosition = false;
    public boolean isAtTargetSpeed = false;

    public Rotation2d steerMotorInput = new Rotation2d();
    public double driveMotorInput = 0.0;

    public double steerMotorInputCalculated = 0.0;
    public double driveMotorInputCalculated = 0.0;

    public String controlModeDrive = "Velocity";
    public String controlModeSteer = "Position";
  }

  /**
   * Updates the inputs of the module
   */
  public void updateInputs();

  /**
   * sets the active target for the module with position and speed conrol loop
   * @param targetState the target state for the module
   */
  public void run(SwerveModuleState targetState);

  /**
   * sets the active target for the module with voltage control loop
   * @param targetState the target state for the module
   */
  public void runVoltage(SwerveModuleState targetState);

  /**
   * gets the current state of the module
   */
  public SwerveModuleState getSwerveModuleState();

  /**
   * gets the position of the module
   */
  public SwerveModulePosition getSwerveModulePosition();

  /**
   * sets the idle mode of the module
   * @param isBrakeMode true for brake mode, false for coast mode
   */
  public void setIdleMode(boolean isBrakeMode);

  /**
   * resets the drive encoder
   */
  public void resetDriveEncoder();

}
