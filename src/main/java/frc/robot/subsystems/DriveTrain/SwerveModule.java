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
  private SwerveModuleState currentState; //the current state of this module
  private SwerveModulePosition modulePostion; //the current postion of this module (no need for target postion)

  private Supplier<Double> driveMotorVelocity; //a supplier of the drive motor velocity
  private Supplier<Double> driveMotorPostion; //a supplier of the drive motor postion
  private Supplier<Double> driveMotorCurrent; //a supplier of the drive motor output current
  private Supplier<Double> driveMotorVoltage; //a supplier of the drive motor output voltage

  private Supplier<Double> steerMotorPostion; //a supplier of the steer motor output postion
  private Supplier<Double> steerMotorCurrent; //a supplier of the steer motor output current
  private Supplier<Double> steerMotorVoltage; //a supplier of the steer motor output voltage

  private Consumer<Double> steerMotorPostionInput; //a consumer for the new target of the steer motor postion loop (including the gear ratio)
  private Consumer<Double> driveMotorVelocityInput; //a consumer for the new speed target of the drive motor velocity loop (including the gear ratio and circufrance)
  

  private TalonFX driveMotor; //the drive motor
  private TalonFXConfiguration driveMotorConfig; //the config of the drive motor (used to set the neutral mode of the motor)

  // private CANcoder absEncoder; //the absolute encoder
  private DutyCycleEncoder absEncoder; //the absolute encoder
  private CANSparkMax steerMotor; //the steer motor
  private RelativeEncoder steerEncoder;

  private SwerveModuleInputsAutoLogged inputs;

  @AutoLog
  public static class SwerveModuleInputs{
    double driveMotorInput;
    double steerMotorInput;

    double driveMotorCurrent;
    double driveMotorVoltage;
    double driveMotorPostion;
    double driveMotorVelocity;
    double driveMotorTempture;

    double steerMotorCurrent;
    double steerMotorVoltage;
    double steerMotorPosition;
    double steerMotorTempture;

    double absEncoderAbsPostion;
    double absEncoderPosition;
  }

  /**
   * the constructor of the swerve module
   * @param moduleConstants the constants of the module
   */
  public SwerveModule(Constants.DriveTrain.SwerveModule moduleConstants) {
    this.moduleConstants = moduleConstants;

    targetState = new SwerveModuleState(0, new Rotation2d());
    currentState = targetState;
    modulePostion = new SwerveModulePosition(0, new Rotation2d());

    absEncoder = configDutyCycleEncoder();

    driveMotorConfig = getTalonFXConfiguration();
    driveMotor = configTalonFX(driveMotorConfig);

    steerMotor = configCanSparkMax(true);

    inputs = new SwerveModuleInputsAutoLogged();
  }

 
  /**
   * checks if the module is at the requested postion
   * @return if the module is at the requested postion
   */
  public boolean isAtRequestedPostion(){
    return Math.abs(currentState.speedMetersPerSecond - targetState.speedMetersPerSecond) <= Constants.DriveTrain.Drive.driveMotorPID.getTolerance() //checks if the speed is within tolarnce
    && Math.abs(currentState.angle.getRotations() - targetState.angle.getRotations()) <= Constants.DriveTrain.Steer.steerMotorPID.getTolerance(); //checks if the rotation is within tolarnce
  } 

  /**
   * sets the module to point to the center of the robot
   * @return the new state of the module
   */
  public SwerveModuleState goToXPostion(){
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
    return targetState;
  }

  /**
   * gets the current state of the module
   * @return the current satate of the module
   */
  public SwerveModuleState getCurrentState(){
    return currentState;
  }

  /**
   * get the target state of the module
   * @return the target state of the module
   */
  public SwerveModuleState getTargetState(){
    return targetState;
  }

  /**
   * get the current postion of the module
   * @return the postion of the module
   */
  public SwerveModulePosition getModulePosition(){
    return modulePostion;
  }

  /**
   * use this to set the module state and target for the motors
   * @param targetState the new module state
   */
  public void setModuleState(SwerveModuleState targetState){
    this.targetState = targetState;

    this.targetState.angle = Rotation2d.fromDegrees(minChangeInSteerAngle(this.targetState.angle.getDegrees()));

    driveMotorVelocityInput.accept(this.targetState.speedMetersPerSecond);//gives the drive motor the new input
    steerMotorPostionInput.accept(this.targetState.angle.getRotations()); //sets the new angle for the steer motor

    inputs.driveMotorInput = this.targetState.speedMetersPerSecond; //updates the input given (for logger)
    inputs.steerMotorInput = this.targetState.angle.getRotations(); //udpats the input given (for logger)

    Logger.processInputs(moduleConstants.moduleName.name(), inputs); //updaes logger
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
   * this updates the module state and the module postion, run this function peridocaly
   * @return the updated state of the module
   */
  public void update(){
    currentState.angle = Rotation2d.fromRotations(steerMotorPostion.get());
    currentState.speedMetersPerSecond = driveMotorVelocity.get();

    modulePostion.angle = Rotation2d.fromRotations(steerMotorPostion.get());
    modulePostion.distanceMeters = driveMotorPostion.get();

    inputs.driveMotorPostion = driveMotorPostion.get(); //updates the postion of the drive motor
    inputs.driveMotorCurrent = driveMotorCurrent.get(); //updates the current output of the drive motor
    inputs.driveMotorVoltage = driveMotorVoltage.get(); //updates the voltage output of the drive motor
    inputs.driveMotorVelocity = driveMotorVelocity.get();
    inputs.driveMotorTempture = driveMotor.getDeviceTemp().getValueAsDouble();


    inputs.steerMotorCurrent = steerMotorCurrent.get(); //updates the current output of the steer motor
    inputs.steerMotorVoltage = steerMotorVoltage.get(); //updates the voltage output of the steer motor
    inputs.steerMotorTempture = steerMotor.getMotorTemperature();

    if(steerEncoder != null){
      inputs.steerMotorPosition = steerMotor.getEncoder().getPosition() / Constants.DriveTrain.Steer.steerGearRatio;
    }
    else{
      inputs.steerMotorPosition = steerMotorPostion.get();
    }

    inputs.absEncoderAbsPostion = (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * 360;
    inputs.absEncoderPosition = steerMotorPostion.get();

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

  /**
   * gets the absolute postion of the cancoder
   * @return the absolute postion of the cancoder in rotations
   */
  public double getAbsolutePosition(){
    return absEncoder.get() ;
  }

  private DutyCycleEncoder configDutyCycleEncoder(){
    DutyCycleEncoder encoder = new DutyCycleEncoder(moduleConstants.absoluteEncoderID);
    encoder.setPositionOffset(moduleConstants.absoluteEncoderZeroOffset / 360);

    return encoder;
  }

  /**
   * use this to config the drive motor at the start of the program
   */
  private TalonFX configTalonFX(TalonFXConfiguration config){
    TalonFX talonFX = new TalonFX(moduleConstants.driveMotorID); //creates a new talon fx

    talonFX.getConfigurator().apply(config); //applys the given config

    talonFX.getPosition().setUpdateFrequency(50); //postion is needed more for odemtry

    talonFX.getVelocity().setUpdateFrequency(50); //sets as default
    talonFX.getMotorVoltage().setUpdateFrequency(50); //sets as default
    talonFX.getSupplyCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getDeviceTemp().setUpdateFrequency(50);

    driveMotorVelocity = talonFX.getVelocity().asSupplier(); //sets the new velocity supplier
    driveMotorPostion = talonFX.getPosition().asSupplier(); //sets the new postion supplier
    driveMotorCurrent = talonFX.getSupplyCurrent().asSupplier(); //sets a supplior of the applied current of the motor for logging
    driveMotorVoltage = talonFX.getMotorVoltage().asSupplier(); //sets a supplior of the applied voltage of the motor for logging

    VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    velocityDutyCycle.EnableFOC = false;

    driveMotorVelocityInput = velocity -> talonFX.setControl(velocityDutyCycle.withVelocity(velocity *  Constants.DriveTrain.Drive.driveGearRatio * Constants.DriveTrain.Drive.wheelCircumferenceMeters)); //creats a consumer that sets the target velocity for the motor

    talonFX.optimizeBusUtilization(); //optimizes canbus util

    talonFX.setPosition(0);

    return talonFX; //returns the ready talon fx
  }

  /**
   * creates a config for the talonFX
   * @return
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
    config.Feedback.SensorToMechanismRatio = Constants.DriveTrain.Drive.driveGearRatio / Constants.DriveTrain.Drive.wheelCircumferenceMeters; //chnages the units to m/s

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

    // sparkMax.getEncoder().setPosition(absEncoder.getAbsolutePosition().getValueAsDouble() * Constants.DriveTrain.Steer.steerGearRatio); //place holder, place getabsencoder postion
    // sparkMax.getEncoder().setPosition(0);
    steerMotorCurrent = () -> sparkMax.getOutputCurrent();
    steerMotorVoltage = () -> sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();

    if(!isUsingRleativeEncoder){
      sparkMax.getEncoder().setPosition(absEncoder.get() * Constants.DriveTrain.Steer.steerGearRatio);

      steerMotorPostion = () -> sparkMax.getEncoder().getPosition() / Constants.DriveTrain.Steer.steerGearRatio;
    }
    else{
      steerEncoder = sparkMax.getAlternateEncoder(8192);
      steerEncoder.setPositionConversionFactor(Constants.DriveTrain.Steer.steerGearRatio);
      steerEncoder.setPosition(absEncoder.getAbsolutePosition() * Constants.DriveTrain.Steer.steerGearRatio);

      sparkMax.getPIDController().setFeedbackDevice(steerEncoder);

      steerMotorPostion = () -> steerEncoder.getPosition() / Constants.DriveTrain.Steer.steerGearRatio;
    }

    steerMotorPostionInput = position -> sparkMax.getPIDController().setReference(position * Constants.DriveTrain.Steer.steerGearRatio, ControlType.kPosition);

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60); 
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}
