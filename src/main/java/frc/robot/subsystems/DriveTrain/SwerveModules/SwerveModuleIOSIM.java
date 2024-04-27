package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSIM implements SwerveModuleIO{
  private final String moduleName;

  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;

  private double driveMotorVoltage = 0;
  private double steerMotorVoltage = 0;

  public SwerveModuleIOSIM(String moduleName){
    driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), Constants.DriveTrain.Drive.driveGearRatio, 0.25);

    if (Constants.robotType == Constants.RobotType.DEVELOPMENT) {
      steerMotor = new DCMotorSim(DCMotor.getNEO(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);
    } else {
      steerMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);
    }

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

    Logger.recordOutput("SwerveModules/" + moduleName + "/driveMotorCurrent", driveMotor.getCurrentDrawAmps());
    Logger.recordOutput("SwerveModules/" + moduleName + "/steerMotorCurrent", steerMotor.getCurrentDrawAmps());

    Logger.processInputs(moduleName + " SwerveModule", inputs);
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
