package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volts;

public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private final PIDController drivePIDController = Constants.DriveTrain.Drive.driveMotorPID.createPIDController();
  private final PIDController steerPIDController = Constants.DriveTrain.Steer.steerMotorPID.createPIDController();

  private SwerveModuleState targetState;

  private final ReentrantLock lock = new ReentrantLock();

  public SwerveModule(Constants.DriveTrain.SwerveModule constants) {
    if(RobotBase.isSimulation()){
      if(Constants.robotType == Constants.RobotType.REPLAY) this.io = new SwerveModuleIO() {};
      else this.io = new SwerveModuleIOSIM(constants.moduleName.name());
    }
    else this.io = new SwerveModuleIODevBot(constants);
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
