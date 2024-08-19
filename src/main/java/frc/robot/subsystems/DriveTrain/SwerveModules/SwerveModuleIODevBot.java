package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorMap;
import frc.util.PIDFGains;

public class SwerveModuleIODevBot extends SwerveModuleIO {

    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final CANcoder absEncoder;

    private final VelocityDutyCycle velocityInput = new VelocityDutyCycle(0).withEnableFOC(false);

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
    protected void sendInputsToMotors(double driveMotorRefrence, double steerMotorRefrence) {
        double driveMotorOut = (driveMotorRefrence / constants.wheelCircumferenceMeters) * constants.driveGearRatio;
        double steerMotorOut = steerMotorRefrence * constants.steerGearRatio;

        driveMotor.setControl(velocityInput.withVelocity(driveMotorOut));
        steerMotor.getPIDController().setReference(steerMotorOut, ControlType.kPosition);
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

    void setDriveMotorPID(PIDFGains pidGains) {
        Slot0Configs config = new Slot0Configs();

        config.kP = pidGains.getP();
        config.kI = pidGains.getI();
        config.kD = pidGains.getD();
        config.kV = pidGains.getF();

        driveMotor.getConfigurator().apply(config);
    }

    @Override
    void setSteerMotorPID(PIDFGains pidGains) {
        SparkPIDController pidController = steerMotor.getPIDController();

        pidController.setP(pidGains.getP());
        pidController.setI(pidGains.getI());
        pidController.setD(pidGains.getD());
        pidController.setFF(pidGains.getF());
    }

}
