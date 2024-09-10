// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterIntake extends SubsystemBase {
  private ShooterIntakeIO io;
  private ShooterIntakeInputsAutoLogged inputs = new ShooterIntakeInputsAutoLogged();

  private double intakeSetSpeed;
  private double onPivotShooterSetSpeed;
  private double offPivotShooterSetSpeed;

  boolean isGpLoaded = false;
  /** Creates a new ShooterIntake. */
  public ShooterIntake() {}
  
  @Override
  public void periodic() {
    io.update(inputs);
    // This method will be called once per scheduler run
    Logger.processInputs("Shooter Intake", inputs);
  }
  public boolean isGpLoaded(){
    return isGpLoaded;
  }
  public void setGpLoaded(boolean gpLoaded){
    this.isGpLoaded = gpLoaded;
  }

  public void setIntakeMotorTargetPosition(double rotation){
    io.setIntakeTargetPosition(rotation);
  }

  public double getIntakeMotorPositions(){
    return inputs.intakeMotorPosition;
  }

  public void setTargetRPMShooterMotorOnPivot(double speed){
    this.onPivotShooterSetSpeed = speed;
    Logger.recordOutput("Shooter Intake/On Pivot Motor Set speed", speed);
    io.setTargetShooterMotorOnPivotRPM(speed);
  }
  public void setTargetRPMShooterMotorOffPivot(double speed){
    this.offPivotShooterSetSpeed = speed;
    io.setTargetShooterMotorOffPivotRPM(speed);
  }
  public void setTargetIntakeMotorRPM(double speed){
    this.intakeSetSpeed = speed;  
    io.setTargetIntakeMotorRPM(speed);
  }
  
  public boolean getBeamBreakValue(){
    return inputs.BeamBreakValue;
  }
  public void StopMotorOnPivot(){
    io.StopMotorOnPivot();
  }
   public void stopMotorOffPivot(){
    io.StopMotorOffPivot();
  }
  public void stopIntakeMotor(){
    io.stopIntakeMotor();
  }
  public double getIntakeMotorRPM(){
    return inputs.intakeMotorRPM;
  }
  
  public boolean isIntakeMotorsUnderLoad(){
    return inputs.intakeMotorCurrent >= ShooterIntakeConstants.INTAKE_MOTOR_UNDER_LOAD_CURRENT;
    
  }

  public boolean isShooterMotorsUnderLoad(){
    return inputs.onPivotShooterMotorCurrent >= ShooterIntakeConstants.SHOOTER_MOTOR_UNDER_LOAD_CURRENT 
    || inputs.offPivotShooterMotorCurrent >= ShooterIntakeConstants.SHOOTER_MOTOR_UNDER_LOAD_CURRENT;
  }

  public boolean isShooterMotorsAtSetSpeed(){

    return Math.abs(inputs.onPivotShooterMotorRPM - onPivotShooterSetSpeed)<= (ShooterIntakeConstants.SHOOTERSPEED) &&
     Math.abs(inputs.offPivotShooterMotorRPM - offPivotShooterSetSpeed)<= (ShooterIntakeConstants.SHOOTERSPEED);

  }

}

