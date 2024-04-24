package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.util.AbsEncoders.AbsEncoderIO;
import frc.util.AbsEncoders.CanCoderIO;


import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volt;

public class SwerveModuleREAL implements SwerveModuleIO{
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final SwerveModule constants;

  private final CANSparkMax steerMotor;
  private final TalonFX driveMotor;

  private final VelocityDutyCycle driveMotorInput = new VelocityDutyCycle(0, 0.0, false, 0.0, 0, false, false, false);

  private final PIDController steerMotorVoltageController;
  private final PIDController driveMotorVoltageController;

  private final AbsEncoderIO absEncoder;

  private final Lock moduleStateLock = new ReentrantLock(true);
  private final Lock targetstateLock = new ReentrantLock(true);

  public SwerveModuleREAL(SwerveModule constants) {
    this.constants = constants;

    absEncoder = new CanCoderIO(constants.AbsEncoderID, constants.isAbsEncoderInverted, 1, constants.absoluteEncoderZeroOffset, false);

    steerMotor = configCanSparkMax();
    driveMotor = configTalonFX(getTalonFXConfiguration());

    steerMotorVoltageController = Constants.DriveTrain.Steer.steerMotorPID.createPIDController();
    driveMotorVoltageController = Constants.DriveTrain.Drive.driveMotorPID.createPIDController();
  }

  @Override
  public SwerveModulePosition modulePeriodic() {

    SwerveModulePosition position = new SwerveModulePosition();
    try {
      moduleStateLock.lock();
      steerMotor.getEncoder().setPosition(absEncoder.getPosition() * Constants.DriveTrain.Steer.steerGearRatio);

      inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / Constants.DriveTrain.Steer.steerGearRatio);
      inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity().getValueAsDouble() / Constants.DriveTrain.Drive.driveGearRatio / Constants.DriveTrain.Drive.wheelCircumferenceMeters;

      position.angle = inputs.currentState.angle;
      position.distanceMeters = driveMotor.getPosition().getValueAsDouble() / Constants.DriveTrain.Drive.driveGearRatio / Constants.DriveTrain.Drive.wheelCircumferenceMeters;
    }
    finally {
      moduleStateLock.unlock();
    }

    try{
      targetstateLock.lock();

      if(inputs.isUsingVoltageController){
        inputs.driveMotorInput = driveMotorVoltageController.calculate(inputs.currentState.speedMetersPerSecond, inputs.targetState.speedMetersPerSecond);
        inputs.steerMotorInput = steerMotorVoltageController.calculate(inputs.currentState.angle.getRadians(), inputs.targetState.angle.getRadians());

        driveMotor.setVoltage(inputs.driveMotorInput);
        steerMotor.setVoltage(inputs.steerMotorInput);
      }

      inputs.isAtTargetPosition = Math.abs(inputs.currentState.angle.getRadians() - inputs.targetState.angle.getRadians()) < Constants.DriveTrain.Steer.steerMotorPID.getTolerance();
      inputs.isAtTargetSpeed = Math.abs(inputs.currentState.speedMetersPerSecond - inputs.targetState.speedMetersPerSecond) < Constants.DriveTrain.Drive.driveMotorPID.getTolerance();
    }
    finally {
      targetstateLock.unlock();
    }

    return position;
  }

  @Override
  public SwerveModuleState run(SwerveModuleState targetState) {
    inputs.driveMotorInput = targetState.speedMetersPerSecond;
    inputs.steerMotorInput = targetState.angle.getRotations();

    SwerveModuleState.optimize(targetState, inputs.currentState.angle);

    targetState.speedMetersPerSecond = targetState.speedMetersPerSecond * Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

    if(!inputs.isUsingVoltageController){
      driveMotor.setControl(driveMotorInput.withVelocity(targetState.speedMetersPerSecond * Constants.DriveTrain.Drive.driveGearRatio * Constants.DriveTrain.Drive.wheelCircumferenceMeters));
      steerMotor.getPIDController().setReference(targetState.angle.getRotations() * Constants.DriveTrain.Steer.steerGearRatio, CANSparkBase.ControlType.kPosition);
    }

    try{
      targetstateLock.lock();
      this.inputs.targetState = targetState;
    }
    finally {
      targetstateLock.unlock();
    }

    return targetState;
  }

  @Override
  public void setIsUsingVoltageController(boolean isUsingVoltageController) {
    inputs.isUsingVoltageController = isUsingVoltageController;
  }

  @Override
  public SwerveModuleState getSwerveModuleState() {
    return inputs.currentState;
  }

  @Override
  public void runSysID(Measure<Voltage> driveVoltage,Measure<Voltage> steerVoltage) {

    if (driveVoltage != null) {
      driveMotor.setVoltage(driveVoltage.in(Volt));
    } else {
      driveMotor.setVoltage(0);
    }

    if (steerVoltage != null) {
      steerMotor.setVoltage(steerVoltage.in(Volt));
    } else {
      steerMotor.setVoltage(0);
    }
  }

  @Override
  public void setIdleMode(boolean isBrakeMode) {
    driveMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    steerMotor.setIdleMode(isBrakeMode ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }

  @Override
  public Lock getModuleState() {
    return moduleStateLock;
  }

  @Override
  public void resetDriveEncoder() {
    driveMotor.setPosition(0);
  }

  /**
   * use this to config the drive motor at the start of the program
   */
  private TalonFX configTalonFX(TalonFXConfiguration config){
    TalonFX talonFX = new TalonFX(constants.driveMotorID); //creates a new talon fx

    talonFX.getConfigurator().apply(config); //apply the given config

    talonFX.getPosition().setUpdateFrequency(SwerveModule.modulesThreadHz); //position is needed more for destroy

    talonFX.getVelocity().setUpdateFrequency(SwerveModule.modulesThreadHz); //sets as default
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

    config.MotorOutput.Inverted = constants.isDriveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //sets the inverted value

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentThreshold = 60;
    config.CurrentLimits.SupplyTimeThreshold = 0.1;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //sets it to coast (changed when the robot is enabled)

    config.Slot0.kP = Constants.DriveTrain.Drive.driveMotorPID.getP(); //sets the P
    config.Slot0.kI = Constants.DriveTrain.Drive.driveMotorPID.getI(); //sets the I
    config.Slot0.kD = Constants.DriveTrain.Drive.driveMotorPID.getD(); //sets the D
    config.Slot0.kS = Constants.DriveTrain.Drive.driveMotorPID.getF(); //sets the feedForward

    config.MotionMagic.MotionMagicAcceleration = Constants.DriveTrain.Drive.driveMotorMaxAcceleration; //sets the max accel for motion magic
    config.MotionMagic.MotionMagicJerk = Constants.DriveTrain.Drive.driveMotorMaxJerk; //sets the max Jerk for motion magic

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //just in case sets the built-in sensor
    config.Feedback.SensorToMechanismRatio = Constants.DriveTrain.Drive.driveGearRatio / Constants.DriveTrain.Drive.wheelCircumferenceMeters; //changes the units to m/s

    return config; //returns the new config
  }

  /**
   * use this to config the steer motor at the start of the program
   */
  private CANSparkMax configCanSparkMax(){
    CANSparkMax sparkMax = new CANSparkMax(constants.steerMotorID, CANSparkLowLevel.MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();

    sparkMax.enableVoltageCompensation(12); //sets voltage compensation to 12V
    sparkMax.setInverted(constants.isSteerInverted); //sets if the motor is inverted or not

    sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast); //sets the idle mode to coat (automatically goes to brakes once the robot is enabled)

    sparkMax.getPIDController().setP(Constants.DriveTrain.Steer.steerMotorPID.getP()); //sets the P for the PID Controller
    sparkMax.getPIDController().setI(Constants.DriveTrain.Steer.steerMotorPID.getI()); //sets the I for the PID Controller
    sparkMax.getPIDController().setD(Constants.DriveTrain.Steer.steerMotorPID.getD()); //sets the D for the PID Controller
    sparkMax.getPIDController().setIZone(Constants.DriveTrain.Steer.steerMotorPID.getIZone()); //sets the IZone for the PID Controller

    sparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.DriveTrain.Steer.steerMotorPID.getTolerance(), 0); //set the tolerance for the module angle
    sparkMax.getPIDController().setSmartMotionMaxAccel(Constants.DriveTrain.Steer.maxAcceleration, 0); //set the max acceleration of the module angle
    sparkMax.getPIDController().setSmartMotionMaxVelocity(Constants.DriveTrain.Steer.maxVelocity, 0); //set the max velocity of the module angle
    sparkMax.getPIDController().setSmartMotionMinOutputVelocity(Constants.DriveTrain.Steer.minVelocity, 0); //set the min velocity of the module angle

    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, (int)((1 / SwerveModule.modulesThreadHz) * 1000)); //sets the status 0 frame to 10ms

    sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module

    sparkMax.getEncoder().setPosition(absEncoder.getAbsolutePosition() * Constants.DriveTrain.Steer.steerGearRatio); //sets the position of the motor to the absolute encoder

//    steerMotorPositionInput = position -> sparkMax.getPIDController().setReference(position * Constants.DriveTrain.Steer.steerGearRatio, CANSparkBase.ControlType.kPosition);

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60);
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}
