package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volts;

public class SwerveModule {
  public static class SwerveModuleConstants{
    public static final double moduleThreadHz = 200;
  }

  private final String moduleName;
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private final ProfiledPIDController drivePIDController;
  private final PIDController steerPIDController;

  private SwerveModuleState targetState;

  private final ReentrantLock lock = new ReentrantLock();

  public SwerveModule(Constants.DriveTrain.SwerveModule constants) {
    if(Constants.robotType == Constants.RobotType.DEVELOPMENT){
      drivePIDController = SwerveModuleIODevBot.DevBotConstants.driveMotorPID.createProfiledPIDController();
      steerPIDController = SwerveModuleIODevBot.DevBotConstants.steerMotorPID.createPIDController();
    }
    else if(Constants.robotType == Constants.RobotType.COMPETITION){
      drivePIDController = SwerveModuleIOCompBot.CompBotConstants.driveMotorPID.createProfiledPIDController();
      steerPIDController = SwerveModuleIOCompBot.CompBotConstants.steerMotorPID.createPIDController();
    }
    else{
      drivePIDController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0), 0);
      steerPIDController = new PIDController(0, 0, 0);
    }

    if(RobotBase.isSimulation()){
      if(Constants.robotType == Constants.RobotType.REPLAY) this.io = new SwerveModuleIO() {};
      else this.io = new SwerveModuleIOSIM();

      SmartDashboard.putData("drivePIDController", drivePIDController);
      SmartDashboard.putData("steerPIDController", steerPIDController);
    }
    else if(Constants.robotType == Constants.RobotType.DEVELOPMENT) this.io = new SwerveModuleIODevBot(constants);
    else this.io = new SwerveModuleIOCompBot(constants);

    this.moduleName = constants.moduleName.name();
  }

  public SwerveModulePosition modulePeriodic() {
    try{
      lock.lock();
      io.updateInputs(inputs);

      if(targetState != null){
        targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
        targetState.speedMetersPerSecond *= Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

        io.setDriveMotorVoltage(drivePIDController.calculate(inputs.currentState.speedMetersPerSecond, targetState.speedMetersPerSecond));
        io.setSteerMotorVoltage(steerPIDController.calculate(inputs.currentState.angle.getRadians(), targetState.angle.getRadians()));
      }
      return new SwerveModulePosition(inputs.drivePositionMeters, inputs.currentState.angle);
    }
    finally {
      lock.unlock();
    }
  }

  public SwerveModuleState getCurrentState(){
    try {
      lock.lock();
      Logger.processInputs(moduleName + " SwerveModule", inputs);
      return inputs.currentState;
    }
    finally {
      lock.unlock();
    }
  }

  public SwerveModuleState run(SwerveModuleState targetState){
    try{
      lock.lock();
      targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
      targetState.speedMetersPerSecond *= inputs.currentState.angle.minus(targetState.angle).getCos();

      this.targetState = targetState;

      return targetState;
    }
    finally {
      lock.unlock();
    }
  }

  public void runSysID(Measure<Voltage> driveVoltage, Measure<Voltage> steerVoltage){
    try{
      lock.lock();
      io.setDriveMotorVoltage(driveVoltage.in(Volts));
      io.setSteerMotorVoltage(steerVoltage.in(Volts));
      targetState = null;
    }
    finally {
      lock.unlock();
    }
  }

  public void setIdleMode(boolean isBrakeMode){
    io.setIdleMode(isBrakeMode);
  }

  public void resetDriveEncoder(){
    io.resetDriveEncoder();
  }
}
