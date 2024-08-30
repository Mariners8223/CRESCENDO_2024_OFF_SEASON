// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIntake extends SubsystemBase {

  ShooterIntakeIO io;
  ShooterIntakeInputsAutoLogged inputs;

  boolean isGpLoaded;
  /** Creates a new ShooterIntake. */
  public ShooterIntake() {}
  
  @Override
  public void periodic() {
    io.update(inputs);

    // This method will be called once per scheduler run
  }
  public boolean isGpLoaded(){
    return isGpLoaded;
  }
  public void setGpLoaded(boolean gpLoaded){
    this.isGpLoaded = gpLoaded;
  }

  public void RPMShooterMotorOnPivot(double speed){
    io.setShooterMotorOnPivotRPM(speed);
  }
  public void RPMShooterMotorOffPivot(double speed){
    io.setShooterMotorOffPivotRPM(speed);
  }
  public void PickupMotorRPM(double speed){
    io.setPickupMotorRPM(speed);
  }
  
  public boolean getBeamBreakValue(){
    return inputs.BeamBreakValue;
  }
}
