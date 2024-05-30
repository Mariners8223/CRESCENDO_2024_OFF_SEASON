package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorMap;
import frc.util.PIDFGains;

public class SwerveModuleIODevBot implements SwerveModuleIO{
  public static class DevBotConstants{
    public static final double driveGearRatio =  6.75;
    public static final double steerGearRatio = 12.5;

    public static final double wheelRadiusMeters = 0.0508;
    public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;

    public static final double maxDriveVelocityMetersPerSecond = 2;

    public static final boolean isDriveInverted = false;
    public static final boolean isSteerInverted = true;
    public static final boolean isAbsEncoderInverted = true;

    public static final double front_left_zeroOffset = 0.302; // the offset between the absolute encoder reading on the front left module, in degrees
    public static final double front_right_zeroOffset = -0.44; // the offset between the absolute encoder on the front left module, in degrees
    public static final double back_left_zeroOffset = -0.164; // the offset between the absolute encoder on the back left module, in degrees
    public static final double back_right_zeroOffset = 0.303; // the offset between the absolute encoder on the back right module, in degrees

    public static final PIDFGains driveMotorPID = new PIDFGains(2.89, 0, 0, 1.2, 0.1, 0, 1 / SwerveModule.moduleThreadHz, 3, 100);
    public static final PIDFGains steerMotorPID = new PIDFGains(10, 0, 0, 0.14, 0.1, 0, 1 / SwerveModule.moduleThreadHz);
  }

  private final TalonFX driveMotor;
  private final CANSparkMax steerMotor;
  private final CANcoder absEncoder;

  public SwerveModuleIODevBot(SwerveModule.ModuleName name){
    int driveMotorID = MotorMap.DriveBase.modules[name.ordinal()][0];
    int steerMotorID = MotorMap.DriveBase.modules[name.ordinal()][1];
    int absEncoderID = MotorMap.DriveBase.modules[name.ordinal()][2];

    double zeroOffset = switch (name){
      case Front_Left -> DevBotConstants.front_left_zeroOffset;
      case Front_Right -> DevBotConstants.front_right_zeroOffset;
      case Back_Left -> DevBotConstants.back_left_zeroOffset;
      case Back_Right -> DevBotConstants.back_right_zeroOffset;
    };

    absEncoder = configCANCoder(absEncoderID, zeroOffset);

    driveMotor = configTalonFX(getTalonFXConfiguration(), driveMotorID);
    steerMotor = configCanSparkMax(steerMotorID);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    // inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.getPosition().getValueAsDouble());
    inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / DevBotConstants.steerGearRatio);

    inputs.currentState.speedMetersPerSecond = (driveMotor.getVelocity().getValueAsDouble() / DevBotConstants.driveGearRatio) * DevBotConstants.wheelCircumferenceMeters;

    inputs.absEncoderPosition = (absEncoder.getAbsolutePosition().getValueAsDouble());

    inputs.steerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / DevBotConstants.steerGearRatio);
    inputs.drivePositionMeters = (driveMotor.getPosition().getValueAsDouble() / DevBotConstants.driveGearRatio) * DevBotConstants.wheelCircumferenceMeters;

    inputs.driveMotorAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.steerMotorAppliedVoltage = steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    steerMotor.setVoltage(voltage);
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


  private CANcoder configCANCoder(int absEncoderID, double absoluteEncoderZeroOffset){
    CANcoder canCoder = new CANcoder(absEncoderID);

    canCoder.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.FutureProofConfigs = false;

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = DevBotConstants.isAbsEncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = -absoluteEncoderZeroOffset;

    canCoder.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz);
    canCoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());

    canCoder.getConfigurator().apply(config);
    canCoder.optimizeBusUtilization();

    return canCoder;
  }

  private TalonFX configTalonFX(TalonFXConfiguration config, int driveMotorID){
    TalonFX talonFX = new TalonFX(driveMotorID); //creates a new talon fx

    talonFX.getConfigurator().apply(config); //apply the given config

    talonFX.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz); //position is needed more for destroy

    talonFX.getVelocity().setUpdateFrequency(SwerveModule.moduleThreadHz); //sets as default
    talonFX.getMotorVoltage().setUpdateFrequency(50); //sets as default
    talonFX.getSupplyCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getStatorCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getDeviceTemp().setUpdateFrequency(50);

    talonFX.optimizeBusUtilization(); //optimizes canbus util

    talonFX.setPosition(0);

    return talonFX; //returns the ready talon fx
  }

  /**
   * creates a config for the talonFX
   * @return the new config
   */
  private TalonFXConfiguration getTalonFXConfiguration(){
    TalonFXConfiguration config = new TalonFXConfiguration(); //creates a new talonFX config

    config.FutureProofConfigs = false; //disables future proofing
    config.Audio.AllowMusicDurDisable = false;

    config.MotorOutput.Inverted = DevBotConstants.isDriveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //sets the inverted value

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentThreshold = 60;
    config.CurrentLimits.SupplyTimeThreshold = 0.1;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //sets it to coast (changed when the robot is enabled)

    config.Slot0.kP = DevBotConstants.driveMotorPID.getP(); //sets the P
    config.Slot0.kI = DevBotConstants.driveMotorPID.getI(); //sets the I
    config.Slot0.kD = DevBotConstants.driveMotorPID.getD(); //sets the D
    config.Slot0.kS = DevBotConstants.driveMotorPID.getF(); //sets the feedForward

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //just in case sets the built-in sensor
    config.Feedback.SensorToMechanismRatio = 1; //changes the units to m/s

    return config; //returns the new config
  }

  /**
   * use this to config the steer motor at the start of the program
   */
  private CANSparkMax configCanSparkMax(int steerMotorID){
    CANSparkMax sparkMax = new CANSparkMax(steerMotorID, CANSparkLowLevel.MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();

    sparkMax.enableVoltageCompensation(12); //sets voltage compensation to 12V
    sparkMax.setInverted(DevBotConstants.isSteerInverted); //sets if the motor is inverted or not

    sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast); //sets the idle mode to coat (automatically goes to brakes once the robot is enabled)

    sparkMax.getPIDController().setP(DevBotConstants.steerMotorPID.getP()); //sets the P for the PID Controller
    sparkMax.getPIDController().setI(DevBotConstants.steerMotorPID.getI()); //sets the I for the PID Controller
    sparkMax.getPIDController().setD(DevBotConstants.steerMotorPID.getD()); //sets the D for the PID Controller
    sparkMax.getPIDController().setIZone(DevBotConstants.steerMotorPID.getIZone()); //sets the IZone for the PID Controller

    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, (int)((1 / SwerveModule.moduleThreadHz) * 1000)); //sets the status 0 frame to 10ms
    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, (int)((1 / SwerveModule.moduleThreadHz) * 1000)); //sets the status 0 frame to 10ms
    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, (int)((1 / SwerveModule.moduleThreadHz) * 1000));

    sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module
    sparkMax.getEncoder().setVelocityConversionFactor(1); //sets the velocity to rad per sec of the module

    sparkMax.getEncoder().setPosition(absEncoder.getPosition().getValueAsDouble() * DevBotConstants.steerGearRatio); //sets the position of the motor to the absolute encoder

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60);
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}