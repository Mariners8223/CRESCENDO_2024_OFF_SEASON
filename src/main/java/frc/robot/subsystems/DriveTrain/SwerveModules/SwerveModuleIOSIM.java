package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

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
    }

    @Override
    protected void sendInputsToMotors(double driveMotorVoltage, double steerMotorVoltage) {
        this.driveMotorVoltage = MathUtil.clamp(driveMotorVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
        this.steerMotorVoltage = MathUtil.clamp(steerMotorVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());

        driveMotor.setInputVoltage(this.driveMotorVoltage);
        steerMotor.setInputVoltage(this.steerMotorVoltage);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        DriverStation.reportWarning("dummy this is a simulation", false);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setState(0, 0);
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
    }
}
