package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
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

    public String controlMode = "MotorController-loop";
  }

  /**
   * Updates the inputs of the module
   */
  public void modulePeriodic();

  /**
   * sets the active target for the module with position and speed control loop
   * @param targetState the target state for the module
   * @return the optimized target state
   */
  public SwerveModuleState run(SwerveModuleState targetState);

  /**
   * sets the active target for the module with voltage control loop
   * @param targetState the target state for the module
   * @return the optimized target state
   */
  public SwerveModuleState runVoltage(SwerveModuleState targetState);

  /**
   * gets the current state of the module
   */
  public SwerveModuleState getSwerveModuleState();

  /**
   * gets the position of the module
   */
  public SwerveModulePosition getSwerveModulePosition();

  /**
   * runs the system identification on the module
   * @param driveVoltage the voltage to run the drive motor at
   * @param steerVoltage the voltage to run the steer motor at
   */
  public void runSysID(Measure<Voltage> driveVoltage,Measure<Voltage> steerVoltage);

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
