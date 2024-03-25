// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class SwerveModule{
  Constants.DriveTrain.SwerveModule moduleConstants; //the constants of this module

  private SwerveModuleState targetState; //the target state of this module
  private final SwerveModuleState currentState; //the current state of this module
  private final SwerveModulePosition modulePosition; //the current position of this module (no need for target position)

  private Supplier<Double> driveMotorVelocity; //a supplier of the drive motor velocity
  private Supplier<Double> driveMotorPosition; //a supplier of the drive motor position
  private Supplier<Double> driveMotorCurrent; //a supplier of the drive motor output current
  private Supplier<Double> driveMotorVoltage; //a supplier of the drive motor output voltage

  private Supplier<Double> steerMotorPosition; //a supplier of the steer motor output position
  private Supplier<Double> steerMotorCurrent; //a supplier of the steer motor output current
  private Supplier<Double> steerMotorVoltage; //a supplier of the steer motor output voltage

  private Consumer<Double> steerMotorPositionInput; //a consumer for the new target of the steer motor position loop (including the gear ratio)
  private Consumer<Double> driveMotorVelocityInput; //a consumer for the new speed target of the drive motor velocity loop (including the gear ratio and circumstance)
  

  private final TalonFX driveMotor; //the drive motor
  private final TalonFXConfiguration driveMotorConfig; //the config of the drive motor (used to set the neutral mode of the motor)
  private final DutyCycleEncoder absEncoder; //the absolute encoder
  private final CANSparkMax steerMotor; //the steer motor
  private RelativeEncoder steerEncoder;

  private final SwerveModuleInputsAutoLogged inputs;


  @AutoLog
  public static class SwerveModuleInputs{
    protected boolean isModuleAtPosition;
    protected boolean isModuleAtSpeed;

    protected double driveMotorInput;
    protected double steerMotorInput;

    protected double driveMotorCurrent;
    protected double driveMotorVoltage;
    protected double driveMotorPosition;
    protected double driveMotorVelocity;
    protected double driveMotorTemperature;

    protected double steerMotorCurrent;
    protected double steerMotorVoltage;
    protected double steerMotorPosition;
    protected double steerMotorTemperature;

    protected double absEncoderAbsPosition;
    protected double absEncoderPosition;
  }

  /**
   * the constructor of the swerve module
   * @param moduleConstants the constants of the module
   */
  public SwerveModule(Constants.DriveTrain.SwerveModule moduleConstants) {
    this.moduleConstants = moduleConstants;

    targetState = new SwerveModuleState(0, new Rotation2d());
    currentState = targetState;
    modulePosition = new SwerveModulePosition(0, new Rotation2d());

    absEncoder = configDutyCycleEncoder();

    driveMotorConfig = getTalonFXConfiguration();
    driveMotor = configTalonFX(driveMotorConfig);

    steerMotor = configCanSparkMax(true);

    inputs = new frc.robot.subsystems.DriveTrain.SwerveModuleInputsAutoLogged();
  }

 
  /**
   * checks if the module is at the requested position
   * @return if the module is at the requested position
   */
  public boolean isAtRequestedPosition(){
    return Math.abs(currentState.angle.getRotations() - targetState.angle.getRotations()) <= Constants.DriveTrain.Steer.steerMotorPID.getTolerance(); //checks if the rotation is within tolerance
  }

    /**
     * checks if the module is at the requested speed
     * @return if the module is at the requested speed
     */
  public boolean isAtRequestedSpeed(){
    return Math.abs(currentState.speedMetersPerSecond - targetState.speedMetersPerSecond) <= Constants.DriveTrain.Drive.driveMotorPID.getTolerance();
  }

  /**
   * sets the module to point to the center of the robot
   */
  public void goToXPosition(){
    driveMotorVelocityInput.accept(0.0); //stops the drive motor
    targetState.speedMetersPerSecond = 0;

    if(Math.abs(currentState.angle.getRotations()) > 1){
      if(moduleConstants.moduleName.ordinal() == 0 || moduleConstants.moduleName.ordinal() == 3){
        targetState.angle = Rotation2d.fromRotations(0.125 * Math.abs((int)currentState.angle.getRotations()));
      }
      else{ 
        targetState.angle = Rotation2d.fromRotations(-0.125 * Math.abs((int)currentState.angle.getRotations()));
      }
    }
    else{
      if(moduleConstants.moduleName.ordinal() == 0 || moduleConstants.moduleName.ordinal() == 3){
        targetState.angle = Rotation2d.fromRotations(0.125);
      }
      else{ 
        targetState.angle = Rotation2d.fromRotations(-0.125);
      }
    }
    targetState = SwerveModuleState.optimize(targetState, currentState.angle);
    setModuleState(targetState);
  }

  /**
   * gets the current state of the module
   * @return the current state of the module
   */
  public SwerveModuleState getCurrentState(){
    return currentState;
  }

  /**
   * get the current position of the module
   * @return the position of the module
   */
  public SwerveModulePosition getModulePosition(){
    return modulePosition;
  }

  /**
   * use this to set the module state and target for the motors
   * @param targetState the new module state
   */
  public void setModuleState(SwerveModuleState targetState){
    this.targetState = targetState;

    this.targetState.angle = Rotation2d.fromDegrees(minChangeInSteerAngle(this.targetState.angle.getDegrees()));

    driveMotorVelocityInput.accept(this.targetState.speedMetersPerSecond);//gives the drive motor the new input
    steerMotorPositionInput.accept(this.targetState.angle.getRotations()); //sets the new angle for the steer motor

    inputs.driveMotorInput = this.targetState.speedMetersPerSecond; //updates the input given (for logger)
    inputs.steerMotorInput = this.targetState.angle.getRotations(); //updates the input given (for logger)

    Logger.processInputs(moduleConstants.moduleName.name(), inputs); //updates logger
  }

  private double minChangeInSteerAngle(double angle) {
    double full_rotations = (int)currentState.angle.getRotations();
    double close_angle = angle + 360.0 * full_rotations;
    double angle_plus = close_angle + 360;
    double angle_minus = close_angle - 360;

    double minAngle = close_angle;
    if(Math.abs(minAngle - currentState.angle.getDegrees()) > Math.abs(angle_plus - currentState.angle.getDegrees())) minAngle = angle_plus;
    if(Math.abs(minAngle - currentState.angle.getDegrees()) > Math.abs(angle_minus - currentState.angle.getDegrees())) minAngle = angle_minus;

    return minAngle;
  }

  /**
   * this updates the module state and the module position, run this function periodically
   */
  public void update(){
    currentState.angle = Rotation2d.fromRotations(steerMotorPosition.get());
    currentState.speedMetersPerSecond = driveMotorVelocity.get();

    modulePosition.angle = Rotation2d.fromRotations(steerMotorPosition.get());
    modulePosition.distanceMeters = driveMotorPosition.get();

    inputs.driveMotorPosition = driveMotorPosition.get(); //updates the position of the drive motor
    inputs.driveMotorCurrent = driveMotorCurrent.get(); //updates the current output of the drive motor
    inputs.driveMotorVoltage = driveMotorVoltage.get(); //updates the voltage output of the drive motor
    inputs.driveMotorVelocity = driveMotorVelocity.get();
    inputs.driveMotorTemperature = driveMotor.getDeviceTemp().getValueAsDouble();


    inputs.steerMotorCurrent = steerMotorCurrent.get(); //updates the current output of the steer motor
    inputs.steerMotorVoltage = steerMotorVoltage.get(); //updates the voltage output of the steer motor
    inputs.steerMotorTemperature = steerMotor.getMotorTemperature();

    if(steerEncoder != null){
      inputs.steerMotorPosition = steerMotor.getEncoder().getPosition() / Constants.DriveTrain.Steer.steerGearRatio;
    }
    else{
      inputs.steerMotorPosition = steerMotorPosition.get();
    }

    inputs.absEncoderAbsPosition = (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * 360;
    inputs.absEncoderPosition = steerMotorPosition.get();

    inputs.isModuleAtPosition = isAtRequestedPosition(); //updates if the module is at the requested position
    inputs.isModuleAtSpeed = isAtRequestedSpeed(); //updates if the module is at the requested speed

    Logger.processInputs(moduleConstants.moduleName.name(), inputs); //updates the logger
  }

  /**
   * sets wheter the motors should be in brake or coast mode
   * @param mode if the motors should be in brake mode
   */
  public void setBrakeMode(boolean mode){
    if(mode){
      steerMotor.setIdleMode(IdleMode.kBrake); //sets the steer motor to brake 
      driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //sets the drive motor to brake
    }
    else{
      steerMotor.setIdleMode(IdleMode.kCoast); //sets the steer motor to rlease (coast)
      driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; //sets the drive motor to rlease  (coast)
    }
    driveMotor.getConfigurator().apply(driveMotorConfig); //applys the new config to the drive motor
  }

  /**
   * sets the drive motor to 0
   */
  public void resetDriveEncoder(){
    driveMotor.setPosition(0);
  }

  private DutyCycleEncoder configDutyCycleEncoder(){
    DutyCycleEncoder encoder = new DutyCycleEncoder(moduleConstants.AbsEncoderPort);
    encoder.setPositionOffset(moduleConstants.absoluteEncoderZeroOffset / 360);

    return encoder;
  }

  /**
   * use this to config the drive motor at the start of the program
   */
  private TalonFX configTalonFX(TalonFXConfiguration config){
    TalonFX talonFX = new TalonFX(moduleConstants.driveMotorID); //creates a new talon fx

    talonFX.getConfigurator().apply(config); //apply the given config

    talonFX.getPosition().setUpdateFrequency(50); //position is needed more for destroy

    talonFX.getVelocity().setUpdateFrequency(50); //sets as default
    talonFX.getMotorVoltage().setUpdateFrequency(50); //sets as default
    talonFX.getSupplyCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getDeviceTemp().setUpdateFrequency(50);

    driveMotorVelocity = talonFX.getVelocity().asSupplier(); //sets the new velocity supplier
    driveMotorPosition = talonFX.getPosition().asSupplier(); //sets the new position supplier
    driveMotorCurrent = talonFX.getSupplyCurrent().asSupplier(); //sets a supplier of the applied current of the motor for logging
    driveMotorVoltage = talonFX.getMotorVoltage().asSupplier(); //sets a supplier of the applied voltage of the motor for logging

    VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    velocityDutyCycle.EnableFOC = false;

    driveMotorVelocityInput = velocity -> talonFX.setControl(velocityDutyCycle.withVelocity(velocity *  Constants.DriveTrain.Drive.driveGearRatio * Constants.DriveTrain.Drive.wheelCircumferenceMeters)); //creates a consumer that sets the target velocity for the motor

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

    config.FutureProofConfigs = false; //disables futre proof
    config.Audio.AllowMusicDurDisable = false;

    if(moduleConstants.isDriveInverted) config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //if the motor is inverted 
    else config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //if motor is not inverted

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
  private CANSparkMax configCanSparkMax(boolean isUsingRleativeEncoder){
    CANSparkMax sparkMax = new CANSparkMax(moduleConstants.steerMotorID, MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();

    sparkMax.enableVoltageCompensation(12); //sets voltage compensation to 12V
    sparkMax.setInverted(moduleConstants.isSteerInverted); //sets wether the motor is inverted or not

    sparkMax.setIdleMode(IdleMode.kCoast); //sets the idle mode to coat (automaticlly goes to brakes once the robot is enabled)

    sparkMax.getPIDController().setP(Constants.DriveTrain.Steer.steerMotorPID.getP()); //sets the P for the PID Controller
    sparkMax.getPIDController().setI(Constants.DriveTrain.Steer.steerMotorPID.getI()); //sets the I for the PID Controller
    sparkMax.getPIDController().setD(Constants.DriveTrain.Steer.steerMotorPID.getD()); //sets the D for the PID Controller
    sparkMax.getPIDController().setIZone(Constants.DriveTrain.Steer.steerMotorPID.getIZone()); //sets the IZone for the PID Controller

    sparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.DriveTrain.Steer.steerMotorPID.getTolerance(), 0); //set the tolarnce for the module angle
    sparkMax.getPIDController().setSmartMotionMaxAccel(Constants.DriveTrain.Steer.maxAcceleration, 0); //set the max acclartion of the module angle
    sparkMax.getPIDController().setSmartMotionMaxVelocity(Constants.DriveTrain.Steer.maxVelocity, 0); //set the max velocity of the module angle
    sparkMax.getPIDController().setSmartMotionMinOutputVelocity(Constants.DriveTrain.Steer.minVelocity, 0); //set the min velocity of the module angle

    sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module

    steerMotorCurrent = sparkMax::getOutputCurrent;
    steerMotorVoltage = () -> sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();

    if(!isUsingRleativeEncoder){
      sparkMax.getEncoder().setPosition(absEncoder.get() * Constants.DriveTrain.Steer.steerGearRatio);

      steerMotorPosition = () -> sparkMax.getEncoder().getPosition() / Constants.DriveTrain.Steer.steerGearRatio;
    }
    else{
      steerEncoder = sparkMax.getAlternateEncoder(8192);
      steerEncoder.setPositionConversionFactor(Constants.DriveTrain.Steer.steerGearRatio);
      steerEncoder.setPosition(absEncoder.getAbsolutePosition() * Constants.DriveTrain.Steer.steerGearRatio);

      sparkMax.getPIDController().setFeedbackDevice(steerEncoder);

      steerMotorPosition = () -> steerEncoder.getPosition() / Constants.DriveTrain.Steer.steerGearRatio;
    }

    steerMotorPositionInput = position -> sparkMax.getPIDController().setReference(position * Constants.DriveTrain.Steer.steerGearRatio, ControlType.kPosition);

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60); 
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}
