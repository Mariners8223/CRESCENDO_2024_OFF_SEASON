package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.util.PIDFGains;

public class SwerveModuleIOSIM extends SwerveModuleIO {
    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private double driveMotorVoltage = 0;
    private double steerMotorVoltage = 0;

    public SwerveModuleIOSIM() {
        driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.25 / constants.DRIVE_GEAR_RATIO);
        if (Constants.robotType == Constants.RobotType.DEVELOPMENT) {
            steerMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.25 / constants.STEER_GEAR_RATIO);
        } else {
            steerMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.25);
        }
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        driveMotor.update(1 / SwerveModule.MODULE_THREAD_HZ);
        steerMotor.update(1 / SwerveModule.MODULE_THREAD_HZ);

        inputs.currentState.speedMetersPerSecond = (driveMotor.getAngularVelocityRadPerSec() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_RADIUS_METERS;
        inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad() / constants.STEER_GEAR_RATIO);

        inputs.steerVelocityRadPerSec = steerMotor.getAngularVelocityRadPerSec() / constants.STEER_GEAR_RATIO;
        inputs.drivePositionMeters = (driveMotor.getAngularPositionRad() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_RADIUS_METERS;

        inputs.driveMotorAppliedVoltage = driveMotorVoltage;
        inputs.steerMotorAppliedVoltage = steerMotorVoltage;

        inputs.driveMotorAppliedOutput = driveMotorVoltage / RobotController.getBatteryVoltage();
        inputs.steerMotorAppliedOutput = steerMotorVoltage / RobotController.getBatteryVoltage();

        inputs.driveMotorRPM = driveMotor.getAngularVelocityRPM();
    }

    @Override
    protected void sendInputsToMotors(double driveMotorReference, double steerMotorReference) {
        double driveMotorVelocity = driveMotor.getAngularVelocityRPM() / 60;
        double steerMotorPosition = steerMotor.getAngularPositionRotations();

        driveMotorReference = (driveMotorReference * driveMotorGearRatio) / driveWheelRadiusMeters;
        steerMotorReference = steerMotorReference * steerMotorGearRatio;

        driveMotorVoltage = driveMotorPIDController.calculate(driveMotorVelocity, driveMotorReference);
        steerMotorVoltage = steerMotorPIDController.calculate(steerMotorPosition, steerMotorReference);

        driveMotor.setInputVoltage(driveMotorVoltage / RobotController.getBatteryVoltage());
        steerMotor.setInputVoltage(steerMotorVoltage / RobotController.getBatteryVoltage());
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        DriverStation.reportWarning("dummy this is a simulation", false);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setState(0, 0);
    }

    @Override
    void setDriveMotorPID(PIDFGains pidGains) {
        driveMotorPIDController = pidGains.createPIDController();
    }

    @Override
    void setSteerMotorPID(PIDFGains pidGains) {
        steerMotorPIDController = pidGains.createPIDController();
    }


    /**
     * A fake class for replaying data
     */
    public static class SwerveModuleIOReplay extends SwerveModuleIO {
        @Override
        public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        }

        @Override
        public void setIdleMode(boolean isBrakeMode) {
        }

        @Override
        protected void sendInputsToMotors(double driveMotorVoltage, double steerMotorVoltage) {
        }

        @Override
        public void resetDriveEncoder() {
        }

        @Override
        void setDriveMotorPID(PIDFGains pidGains) {

        }

        @Override
        void setSteerMotorPID(PIDFGains pidGains) {

        }
    }
}
