// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule{
  private final Constants.DriveTrain.SwerveModule moduleConstants; //the constants of this module

  private Consumer<Double> steerMotorPositionInput; //a consumer for the new target of the steer motor position loop (including the gear ratio)
  private Consumer<Double> driveMotorVelocityInput; //a consumer for the new speed target of the drive motor velocity loop (including the gear ratio and circumstance)


  private final SwerveModulePosition modulePosition = new SwerveModulePosition();
  private final TalonFX driveMotor; //the drive motor

  private final CANcoder absEncoder; //the absolute encoder
  private final CANSparkMax steerMotor; //the steer motor

  private final SwerveModuleInputsAutoLogged inputs;


  @AutoLog
  public static class SwerveModuleInputs{

    protected SwerveModuleState currentState = new SwerveModuleState();
    protected SwerveModuleState targetState = new SwerveModuleState();

    protected boolean isModuleAtPosition;
    protected boolean isModuleAtSpeed;

    protected double driveMotorInput;
    protected double steerMotorInput;
    protected double steerMotorInputCurrent;
  }

  /**
   * the constructor of the swerve module
   * @param moduleConstants the constants of the module
   */
  public SwerveModule(Constants.DriveTrain.SwerveModule moduleConstants) {
    this.moduleConstants = moduleConstants;

    absEncoder = configCanCoder();

    TalonFXConfiguration driveMotorConfig = getTalonFXConfiguration();
    driveMotor = configTalonFX(driveMotorConfig);

    steerMotor = configCanSparkMax();

    inputs = new frc.robot.subsystems.DriveTrain.SwerveModuleInputsAutoLogged();
  }

 
  /**
   * checks if the module is at the requested position
   * @return if the module is at the requested position
   */
  public boolean isAtRequestedPosition(){
    return Math.abs(inputs.currentState.angle.getRotations() - inputs.targetState.angle.getRotations()) <= Constants.DriveTrain.Steer.steerMotorPID.getTolerance(); //checks if the rotation is within tolerance
  }

    /**
     * checks if the module is at the requested speed
     * @return if the module is at the requested speed
     */
  public boolean isAtRequestedSpeed(){
    return Math.abs(inputs.currentState.speedMetersPerSecond - inputs.targetState.speedMetersPerSecond) <= Constants.DriveTrain.Drive.driveMotorPID.getTolerance();
  }

  /**
   * sets the module to point to the center of the robot
   */
  public void goToXPosition(){
    driveMotorVelocityInput.accept(0.0); //stops the drive motor
    inputs.targetState.speedMetersPerSecond = 0;

    if(Math.abs(inputs.currentState.angle.getRotations()) > 1){
      if(moduleConstants.moduleName.ordinal() == 0 || moduleConstants.moduleName.ordinal() == 3){
        inputs.targetState.angle = Rotation2d.fromRotations(0.125 * Math.abs((int)inputs.currentState.angle.getRotations()));
      }
      else{ 
        inputs.targetState.angle = Rotation2d.fromRotations(-0.125 * Math.abs((int)inputs.currentState.angle.getRotations()));
      }
    }
    else{
      if(moduleConstants.moduleName.ordinal() == 0 || moduleConstants.moduleName.ordinal() == 3){
        inputs.targetState.angle = Rotation2d.fromRotations(0.125);
      }
      else{ 
        inputs.targetState.angle = Rotation2d.fromRotations(-0.125);
      }
    }
    inputs.targetState = SwerveModuleState.optimize(inputs.targetState, inputs.currentState.angle);
    setModuleState(inputs.targetState);
  }

  /**
   * gets the current state of the module
   * @return the current state of the module
   */
  public SwerveModuleState getCurrentState(){
    return inputs.currentState;
  }

  /**
   * get the current position of the module
   * @return the position of the module
   */
  public SwerveModulePosition getModulePosition(){ return modulePosition; }

  /**
   * use this to set the module state and target for the motors
   * @param targetState the new module state
   */
  public void setModuleState(SwerveModuleState targetState){
    this.inputs.targetState = targetState;

//    this.inputs.targetState.angle = Rotation2d.fromDegrees(this.inputs.targetState.angle.getDegrees()); // probably for edens calc IDK why is it here

    driveMotorVelocityInput.accept(this.inputs.targetState.speedMetersPerSecond);//gives the drive motor the new input
    steerMotorPositionInput.accept(this.inputs.targetState.angle.getRotations()); //sets the new angle for the steer motor

    inputs.driveMotorInput = this.inputs.targetState.speedMetersPerSecond; //updates the input given (for logger)
    inputs.steerMotorInput = this.inputs.targetState.angle.getRotations(); //updates the input given (for logger)

    Logger.processInputs(moduleConstants.moduleName.name(), inputs); //updates logger
  }

  /**
   * sets the steer motor voltage
   * @param voltage the voltage to set the motor to
   */
  public void setSteerMotorVoltage(double voltage){
    steerMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
  }

  /**
   * sets the drive motor voltage
   * @param voltage the voltage to set the motor to
   */
  public void setDriveMotorVoltage(double voltage){
    VoltageOut voltageOut = new VoltageOut(voltage);
    voltageOut.EnableFOC = false;
    driveMotor.setControl(voltageOut);
  }

  /**
   * this updates the module state and the module position, run this function periodically
   */
  public void update(){

    if(moduleConstants.shouldSteerMotorBeResetedByAbsEncoder){
      steerMotor.getEncoder().setPosition(absEncoder.getPosition().getValueAsDouble() * Constants.DriveTrain.Steer.steerGearRatio);
      inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.getPosition().getValueAsDouble());
    }
    else{
      inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / Constants.DriveTrain.Steer.steerGearRatio);
    }

    modulePosition.angle = inputs.currentState.angle;
    inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity().getValueAsDouble();
    modulePosition.distanceMeters = driveMotor.getPosition().getValueAsDouble();


    inputs.steerMotorInputCurrent = steerMotor.getOutputCurrent() * steerMotor.getAppliedOutput(); //updates the input current of the steer motor

    inputs.isModuleAtPosition = isAtRequestedPosition(); //updates if the module is at the requested position
    inputs.isModuleAtSpeed = isAtRequestedSpeed(); //updates if the module is at the requested speed

    Logger.processInputs(moduleConstants.moduleName.name(), inputs); //updates the logger
  }

  /**
   * sets if the motors should be in brake or coast mode
   * @param mode if the motors should be in brake mode
   */
  public void setBrakeMode(boolean mode){
    if(mode){
      steerMotor.setIdleMode(IdleMode.kBrake); //sets the steer motor to brake
      driveMotor.setNeutralMode(NeutralModeValue.Brake); //sets the drive motor to brake
    }
    else{
      steerMotor.setIdleMode(IdleMode.kCoast); //sets the steer motor to release (coast)
      driveMotor.setNeutralMode(NeutralModeValue.Coast); //sets the drive motor to coast
    }
  }

  /**
   * sets the drive motor position to 0
   */
  public void resetDriveEncoder(){
    driveMotor.setPosition(0);
  }


  private CANcoder configCanCoder(){
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.FutureProofConfigs = false;

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    config.MagnetSensor.SensorDirection = moduleConstants.isAbsEncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

    config.MagnetSensor.MagnetOffset = moduleConstants.absoluteEncoderZeroOffset;

    CANcoder canCoder = new CANcoder(moduleConstants.AbsEncoderID);

    canCoder.getConfigurator().apply(config);

    canCoder.getAbsolutePosition().setUpdateFrequency(50);
    canCoder.getPosition().setUpdateFrequency(50);

    canCoder.optimizeBusUtilization();

    return canCoder;
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
    talonFX.getStatorCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getDeviceTemp().setUpdateFrequency(50);

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

    config.FutureProofConfigs = false; //disables future proofing
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
  private CANSparkMax configCanSparkMax(){
    CANSparkMax sparkMax = new CANSparkMax(moduleConstants.steerMotorID, MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();

    sparkMax.enableVoltageCompensation(12); //sets voltage compensation to 12V
    sparkMax.setInverted(moduleConstants.isSteerInverted); //sets if the motor is inverted or not

    sparkMax.setIdleMode(IdleMode.kCoast); //sets the idle mode to coat (automatically goes to brakes once the robot is enabled)

    sparkMax.getPIDController().setP(Constants.DriveTrain.Steer.steerMotorPID.getP()); //sets the P for the PID Controller
    sparkMax.getPIDController().setI(Constants.DriveTrain.Steer.steerMotorPID.getI()); //sets the I for the PID Controller
    sparkMax.getPIDController().setD(Constants.DriveTrain.Steer.steerMotorPID.getD()); //sets the D for the PID Controller
    sparkMax.getPIDController().setIZone(Constants.DriveTrain.Steer.steerMotorPID.getIZone()); //sets the IZone for the PID Controller

    sparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(Constants.DriveTrain.Steer.steerMotorPID.getTolerance(), 0); //set the tolerance for the module angle
    sparkMax.getPIDController().setSmartMotionMaxAccel(Constants.DriveTrain.Steer.maxAcceleration, 0); //set the max acceleration of the module angle
    sparkMax.getPIDController().setSmartMotionMaxVelocity(Constants.DriveTrain.Steer.maxVelocity, 0); //set the max velocity of the module angle
    sparkMax.getPIDController().setSmartMotionMinOutputVelocity(Constants.DriveTrain.Steer.minVelocity, 0); //set the min velocity of the module angle

    sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module

    sparkMax.getEncoder().setPosition(absEncoder.getAbsolutePosition().getValueAsDouble() * Constants.DriveTrain.Steer.steerGearRatio); //sets the position of the motor to the absolute encoder

    steerMotorPositionInput = position -> sparkMax.getPIDController().setReference(position * Constants.DriveTrain.Steer.steerGearRatio, ControlType.kPosition);

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60); 
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}
