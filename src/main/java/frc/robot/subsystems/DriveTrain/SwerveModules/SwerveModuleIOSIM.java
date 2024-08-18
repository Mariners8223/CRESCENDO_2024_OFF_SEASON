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

    private final double driveMotorGearRatio = constants.driveGearRatio;
    private final double driveWheelRadiusMeters = constants.wheelRadiusMeters;
    private final double steerMotorGearRatio = constants.steerGearRatio;

    private PIDController driveMotorPIDController;
    private PIDController steerMotorPIDController;

    private double driveMotorVoltage = 0;
    private double steerMotorVoltage = 0;

    public SwerveModuleIOSIM() {

        driveMotorPIDController = constants.driveMotorPID[0].createPIDController();
        steerMotorPIDController = constants.steerMotorPID[0].createPIDController();

        driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.25); //TODO need to find real moment of inertia

        steerMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.25); //TODO need to find real moment of inertia
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        driveMotor.update(1 / SwerveModule.moduleThreadHz);
        steerMotor.update(1 / SwerveModule.moduleThreadHz);

        inputs.currentState.speedMetersPerSecond = (driveMotor.getAngularVelocityRadPerSec() / driveMotorGearRatio) * driveWheelRadiusMeters;
        inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad() / steerMotorGearRatio);

        inputs.steerVelocityRadPerSec = steerMotor.getAngularVelocityRadPerSec() / steerMotorGearRatio;
        inputs.drivePositionMeters = driveMotor.getAngularPositionRad() * driveWheelRadiusMeters;

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
