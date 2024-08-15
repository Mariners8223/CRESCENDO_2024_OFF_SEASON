package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorMap;

public class SwerveModuleIODevBot extends SwerveModuleIO {

    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final CANcoder absEncoder;

    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.MODULES[name.ordinal()][2];

        double zeroOffset = constants.ABSOLUTE_ZERO_OFFSETS[name.ordinal()];

        absEncoder = configCANCoder(absEncoderID, zeroOffset);

        driveMotor = configTalonFX(getTalonFXConfiguration(name), driveMotorID);
        steerMotor = configCanSparkMax(steerMotorID, absEncoder.getAbsolutePosition().getValueAsDouble(), name);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        // inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.getPosition().getValueAsDouble());
        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / constants.STEER_GEAR_RATIO);

        inputs.currentState.speedMetersPerSecond =
                (driveMotor.getVelocity().getValueAsDouble() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_CIRCUMFERENCE_METERS;

        inputs.absEncoderPosition = (absEncoder.getAbsolutePosition().getValueAsDouble());

        inputs.driveMotorRPM = driveMotor.getVelocity().getValueAsDouble() * 60;
        inputs.driveMotorTemperature = driveMotor.getDeviceTemp().getValueAsDouble();

        inputs.steerVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / constants.STEER_GEAR_RATIO);

        inputs.drivePositionMeters =
                (driveMotor.getPosition().getValueAsDouble() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_CIRCUMFERENCE_METERS;

        inputs.driveMotorAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerMotorAppliedVoltage = steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
    }

    @Override
    protected void sendInputsToMotors(double driveMotorVoltageInput, double steerMotorVoltageInput) {
        driveMotor.setVoltage(driveMotorVoltageInput);
        steerMotor.setVoltage(steerMotorVoltageInput);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        driveMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        steerMotor.setIdleMode(isBrakeMode ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setPosition(0);
    }


}
