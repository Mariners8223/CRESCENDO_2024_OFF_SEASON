package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSIM implements SwerveModuleIO{
  private final String moduleName;

  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;

  private double driveMotorVoltage = 0;
  private double steerMotorVoltage = 0;

  public SwerveModuleIOSIM(String moduleName, Constants.RobotType botType){
    driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), Constants.DriveTrain.Drive.driveGearRatio, 0.25);
    steerMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);
//    if (Objects.requireNonNull(botType) == Constants.RobotType.COMPETITION) {
//      steerMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);
//    } else {
//      steerMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);
//    }

    this.moduleName = moduleName;
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    driveMotor.update(1 / Constants.DriveTrain.SwerveModule.modulesThreadHz);
    steerMotor.update(1 / Constants.DriveTrain.SwerveModule.modulesThreadHz);

    inputs.currentState.speedMetersPerSecond = driveMotor.getAngularVelocityRadPerSec() * Constants.DriveTrain.Drive.wheelRadiusMeters;
    inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad());

    inputs.steerVelocityRadPerSec = steerMotor.getAngularVelocityRadPerSec();
    inputs.drivePositionMeters = driveMotor.getAngularPositionRad() * Constants.DriveTrain.Drive.wheelRadiusMeters;

    inputs.driveMotorAppliedVoltage = driveMotorVoltage;
    inputs.steerMotorAppliedVoltage = steerMotorVoltage;

    Logger.processInputs(moduleName + " SwerveModule", inputs);
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    driveMotorVoltage = voltage;
    driveMotor.setInputVoltage(voltage);
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    steerMotorVoltage = voltage;
    steerMotor.setInputVoltage(voltage);
  }

  @Override
  public void setIdleMode(boolean isBrakeMode) {
    DriverStation.reportWarning("dummy this is a simulation", false);
  }

  @Override
  public void resetDriveEncoder() {
    driveMotor.setState(0, 0);
  }
}
