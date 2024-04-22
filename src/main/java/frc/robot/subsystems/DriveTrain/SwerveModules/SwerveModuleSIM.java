package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.*;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.SwerveModule;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveModuleSIM implements SwerveModuleIO{
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final SwerveModulePosition modulePosition;

    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private final Lock lock = new ReentrantLock();

    private final long loopTimeMs;

    public SwerveModuleSIM(SwerveModule constants, long loopTimeMs) {

        this.driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), Constants.DriveTrain.Drive.driveGearRatio, 0.25);
        this.steerMotor = new DCMotorSim(DCMotor.getNeo550(1), Constants.DriveTrain.Steer.steerGearRatio, 0.25);

        this.modulePosition = new SwerveModulePosition(0,new Rotation2d());

        this.loopTimeMs = loopTimeMs;
    }

    @Override
    public void modulePeriodic() {
        driveMotor.update(loopTimeMs / 1000.0);
        steerMotor.update(loopTimeMs / 1000.0);

        try {
            lock.lock();
            inputs.currentState.speedMetersPerSecond = driveMotor.getAngularVelocityRadPerSec() * Constants.DriveTrain.Drive.wheelRadiusMeters;
            inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad());

            this.modulePosition.angle = inputs.currentState.angle;
            this.modulePosition.distanceMeters = driveMotor.getAngularPositionRad() * Constants.DriveTrain.Drive.wheelRadiusMeters;
        }
        finally {
            lock.unlock();
        }
    }

    @Override
    public SwerveModuleState run(SwerveModuleState targetState) {
        return null;
    }

    @Override
    public SwerveModuleState runVoltage(SwerveModuleState targetState) {
        return null;
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
    public void runSysID(Measure<Voltage> driveVoltage, Measure<Voltage> steerVoltage) {

    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {

    }

    @Override
    public Lock getLock() {
        return null;
    }

    @Override
    public void resetDriveEncoder() {

    }
}
