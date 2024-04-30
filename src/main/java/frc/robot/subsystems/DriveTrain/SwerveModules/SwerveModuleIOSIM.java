package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SwerveModuleIOSIM implements SwerveModuleIO{
  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;

  private double driveMotorVoltage = 0;
  private double steerMotorVoltage = 0;

  public SwerveModuleIOSIM(){


    if (Constants.robotType == Constants.RobotType.DEVELOPMENT) {
      steerMotor = new DCMotorSim(DCMotor.getNEO(1), SwerveModuleIODevBot.DevBotConstants.driveGearRatio, 0.25);
      driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), SwerveModuleIODevBot.DevBotConstants.driveGearRatio * SwerveModuleIODevBot.DevBotConstants.wheelRadiusMeters, 0.25);
    } else {
      steerMotor = new DCMotorSim(DCMotor.getNeo550(1), SwerveModuleIOCompBot.CompBotConstants.steerGearRatio, 0.25);
      driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), SwerveModuleIOCompBot.CompBotConstants.driveGearRatio * SwerveModuleIOCompBot.CompBotConstants.wheelRadiusMeters, 0.25);
    }
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    driveMotor.update(1 / SwerveModule.SwerveModuleConstants.moduleThreadHz);
    steerMotor.update(1 / SwerveModule.SwerveModuleConstants.moduleThreadHz);

    inputs.currentState.speedMetersPerSecond = driveMotor.getAngularVelocityRadPerSec();
    inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad());

    inputs.steerVelocityRadPerSec = steerMotor.getAngularVelocityRadPerSec();
    inputs.drivePositionMeters = driveMotor.getAngularPositionRad();

    inputs.driveMotorAppliedVoltage = driveMotorVoltage;
    inputs.steerMotorAppliedVoltage = steerMotorVoltage;
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    driveMotorVoltage = MathUtil.clamp(voltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    driveMotor.setInputVoltage(voltage);
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    steerMotorVoltage = MathUtil.clamp(voltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
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
