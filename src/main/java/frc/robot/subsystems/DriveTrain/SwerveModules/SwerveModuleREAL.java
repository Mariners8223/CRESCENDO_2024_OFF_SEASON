package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.DriveTrain.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleREAL implements SwerveModuleIO{
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final SwerveModule consntants;

  private final SwerveModulePosition modulePosition = new SwerveModulePosition();

  private final CANSparkMax steerMotor;
  private final TalonFX driveMotor;

  public SwerveModuleREAL(SwerveModule consntants) {
    this.consntants = consntants;

    steerMotor = new CANSparkMax(consntants.steerMotorID, CANSparkLowLevel.MotorType.kBrushless);
    driveMotor = new TalonFX(consntants.driveMotorID);
  }

  @Override
  public void updateInputs() {

  }

  @Override
  public void run(SwerveModuleState targetState) {

  }

  @Override
  public void runVoltage(SwerveModuleState targetState) {

  }

  @Override
  public SwerveModuleState getSwerveModuleState() {
    return null;
  }

  @Override
  public SwerveModulePosition getSwerveModulePosition() {
    return null;
  }

  @Override
  public void setIdleMode(boolean isBrakeMode) {

  }

  @Override
  public void resetDriveEncoder() {

  }

  private CANSparkMax configSteerMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setInverted(consntants.steerMotorInverted);
    motor.setOpenLoopRampRate(consntants.steerMotorRampRate);
    motor.setSmartCurrentLimit(consntants.steerMotorCurrentLimit);
    return motor;
  }
}
