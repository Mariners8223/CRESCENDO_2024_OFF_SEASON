// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIntakeIO {

    @AutoLog
    public class ShooterIntakeInputs{
        double RPMShooterMotorOnPivot;
        double RPMShooterMotorOffPivot;
        double PickupMotorRPM;
        boolean BeamBreakValue;
    }
//RPM = rotations per min
    public void setShooterMotorOnPivotRPM(double speedRPM);
    public void setShooterMotorOffPivotRPM(double speedRPM);
    public void setPickupMotorRPM(double speedRPM);


    public void update(ShooterIntakeInputsAutoLogged inputs);

   
}
