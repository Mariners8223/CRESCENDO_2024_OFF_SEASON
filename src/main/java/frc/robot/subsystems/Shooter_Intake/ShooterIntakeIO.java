// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIntakeIO {

    @AutoLog
    public class ShooterIntakeInputs{
        double shit;
        double moreShit;
    }
//SPL = spineless SP = spinefull
//RPM = rotations per min
    public void setMotorSpeedShooterSPRPM(double speedRPM);
    public void setMotorSpeedShooterSPLRPM(double speedRPM);
    public void setMotorSpeedPickupRPM(double speedRPM);
   


    public void update(ShooterIntakeInputsAutoLogged inputs);
    
}
