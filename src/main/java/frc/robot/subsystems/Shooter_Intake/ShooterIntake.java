// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterIntake extends SubsystemBase {
  private ShooterIntakeIO io;
  private ShooterIntakeInputsAutoLogged inputs = new ShooterIntakeInputsAutoLogged();

  private double onPivotShooterSetSpeed;
  private double offPivotShooterSetSpeed;

  boolean isGpLoaded = false;
  /** Creates a new ShooterIntake. */
  public ShooterIntake() {
    io = new ShooterIntakeIOReal();
  }
  
  @Override
  public void periodic() {
    io.update(inputs);
    // This method will be called once per scheduler run
    Logger.processInputs("Shooter Intake", inputs);

    String currentCommandName = getCurrentCommand() != null ? getCurrentCommand().getName() : "none";

    Logger.recordOutput("Shooter/current command", currentCommandName);
  }
  public boolean isGpLoaded(){
    return isGpLoaded;
  }
  public void setGpLoaded(boolean gpLoaded){

    Logger.recordOutput("Shooter/is gp loaded", gpLoaded);

    this.isGpLoaded = gpLoaded;
  }

  /*
   * set amount of rotations the motor needs to do relitive to zero set postion
   * @ param rotation num of rotations or part of rotation to turn, sign denotes direction of turn 
   */ 
  public void setIntakeMotorTargetPosition(double rotation){

    Logger.recordOutput("Intake Motor /Set rotation", rotation);

    io.setIntakeTargetPosition(rotation);
  }

  public double getIntakeMotorPositions(){
    return inputs.intakeMotorPosition;
  }

  public void setTargetRPMShooterMotorOnPivot(double speed){
    this.onPivotShooterSetSpeed = speed;

    Logger.recordOutput("Shooter/ On Pivot Motor /Set speed", speed);

    io.setTargetShooterMotorOnPivotRPM(speed);
  }

  public void setTargetRPMShooterMotorOffPivot(double speed){
    this.offPivotShooterSetSpeed = speed;

    Logger.recordOutput("Shooter/ Off Pivot Motor /Set speed", speed);

    io.setTargetShooterMotorOffPivotRPM(speed);
  }

  public void setIntakeMotorDutyCycle(double dutyCycle){

    Logger.recordOutput("Shooter/ Intake Motor /Set speed", dutyCycle);

    io.setTargetIntakeMotorDutyCycle(dutyCycle);
  }

  public void setTargetShooterMotorOnPivotDutyCycle(double dutyCycle){
    this.onPivotShooterSetSpeed = dutyCycle;

    Logger.recordOutput("Shooter/ On Pivot Motor /Set speed", dutyCycle);

    io.setTargetShooterMotorOnPivotDutyCycle(dutyCycle);
  }

  public void setTargetShooterMotorOffPivotDutyCycle(double dutyCycle){
    this.onPivotShooterSetSpeed = dutyCycle;

    Logger.recordOutput("Shooter/ On Pivot Motor /Set speed", dutyCycle);

    io.setTargetShooterMotorOffPivotDutyCycle(dutyCycle);
  }
  
  
  public boolean getBeamBreakValue(){
    return inputs.BeamBreakValue;
  }
  public void StopMotorOnPivot(){
    Logger.recordOutput("Shooter/ On Pivot Motor /Set speed",0.0);
    io.StopMotorOnPivot();
  }
   public void stopMotorOffPivot(){
    Logger.recordOutput("Shooter/ Off Pivot Motor /Set speed",0.0);
    io.StopMotorOffPivot();
  }
  public void stopIntakeMotor(){
    Logger.recordOutput("Shooter/ Intake Motor /Set speed", 0.0);
    io.stopIntakeMotor();
  }
  public double getIntakeMotorRPM(){
    return inputs.intakeMotorRPM;
  }
  
  public boolean isIntakeMotorUnderLoad(){
    return inputs.intakeMotorCurrent > ShooterIntakeConstants.INTAKE_MOTOR_UNDER_LOAD_CURRENT;
  }

  public boolean isShooterMotorsUnderLoad(){
    return inputs.onPivotShooterMotorCurrent >= ShooterIntakeConstants.SHOOTER_MOTOR_UNDER_LOAD_CURRENT 
    || inputs.offPivotShooterMotorCurrent >= ShooterIntakeConstants.SHOOTER_MOTOR_UNDER_LOAD_CURRENT;
  }

  public boolean isShooterMotorsAtSetSpeed(){
    return Math.abs(inputs.onPivotShooterMotorRPM - onPivotShooterSetSpeed)<= (ShooterIntakeConstants.SHOOTER_SPEED_TOLRANCE) &&
    Math.abs(inputs.offPivotShooterMotorRPM - offPivotShooterSetSpeed)<= (ShooterIntakeConstants.SHOOTER_SPEED_TOLRANCE);
  }

}

