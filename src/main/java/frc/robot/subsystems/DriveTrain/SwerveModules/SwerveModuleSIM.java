package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.SwerveModule;

import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

public class SwerveModuleSIM implements SwerveModuleIO{
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final SwerveModule consants;

    private final SwerveModulePosition modulePosition = new SwerveModulePosition();

    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private final PIDController drivePIDController = Constants.DriveTrain.Drive.driveMotorPID.createPIDController();
    private final PIDController steerPIDController = Constants.DriveTrain.Steer.steerMotorPID.createPIDController();

    private final ReentrantLock currentLock = new ReentrantLock();

    public SwerveModuleSIM(SwerveModule constants) {
        this.consants = constants;

        this.driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), Constants.DriveTrain.Drive.driveGearRatio, 0.25);
        this.steerMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);

        SmartDashboard.putData("Drive PID", drivePIDController);
        SmartDashboard.putData("Steer PID", steerPIDController);
    }

    @Override
    public SwerveModulePosition modulePeriodic() {
        driveMotor.update(1 / SwerveModule.modulesThreadHz);
        steerMotor.update(1 / SwerveModule.modulesThreadHz);

        try {
            currentLock.lock();
            inputs.currentState.speedMetersPerSecond = driveMotor.getAngularVelocityRadPerSec() * Constants.DriveTrain.Drive.wheelRadiusMeters;
            inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad());

            this.modulePosition.angle = inputs.currentState.angle;
            this.modulePosition.distanceMeters = driveMotor.getAngularPositionRad() * Constants.DriveTrain.Drive.wheelRadiusMeters;

            inputs.isAtTargetSpeed = drivePIDController.atSetpoint();
            inputs.isAtTargetPosition = steerPIDController.atSetpoint();

            inputs.driveMotorInput = drivePIDController.calculate(inputs.currentState.speedMetersPerSecond, inputs.targetState.speedMetersPerSecond);
            inputs.steerMotorInput = steerPIDController.calculate(inputs.currentState.angle.getRotations(), inputs.targetState.angle.getRotations());

            driveMotor.setInputVoltage(inputs.driveMotorInput);
            steerMotor.setInputVoltage(inputs.steerMotorInput);

            Logger.processInputs("MODULE " + consants.moduleName, inputs);

            return this.modulePosition;
        }
        finally {
            currentLock.unlock();
        }
    }

    @Override
    public SwerveModuleState run(SwerveModuleState targetState) {

        targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);

        targetState.speedMetersPerSecond = targetState.speedMetersPerSecond * Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

        try {
            currentLock.lock();
            inputs.targetState = targetState;
        }
        finally {
            currentLock.unlock();
        }

        return targetState;
    }

    @Override
    public void setIsUsingVoltageController(boolean isUsingVoltageController) {}

    @Override
    public SwerveModuleState getSwerveModuleState() {
        try{
            currentLock.lock();
            return inputs.currentState;
        }
        finally {
            currentLock.unlock();
        }
    }

    @Override
    public void runSysID(Measure<Voltage> driveVoltage, Measure<Voltage> steerVoltage) {

    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {

    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setState(0.0, 0.0);
    }
}
