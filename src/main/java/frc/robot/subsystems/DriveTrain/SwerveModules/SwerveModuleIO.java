package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

import java.util.concurrent.locks.Lock;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public SwerveModuleState currentState = new SwerveModuleState();
    public SwerveModuleState targetState = new SwerveModuleState();

    public double SteerVelocityRadPerSec = 0.0;
    public double DrivePositionMeters = 0.0;

    public boolean isAtTargetPosition = false;
    public boolean isAtTargetSpeed = false;

    public double steerMotorInput = 0.0;
    public double driveMotorInput = 0.0;

    public boolean isUsingVoltageController = false;
  }

  /**
   * Updates the inputs of the module
   */
  public SwerveModulePosition modulePeriodic();

  /**
   * sets the active target for the module with position and speed control loop
   * @param targetState the target state for the module
   * @return the optimized target state
   */
  public SwerveModuleState run(SwerveModuleState targetState);

  /**
   * sets the type of controller for the module (or voltage controller on rio or position and velocity controller on motor controller)
   * @param isUsingVoltageController true for voltage control, false for speed and position controle
   */
  public void setIsUsingVoltageController(boolean isUsingVoltageController);

  /**
   * gets the current state of the module
   */
  public SwerveModuleState getSwerveModuleState();

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
