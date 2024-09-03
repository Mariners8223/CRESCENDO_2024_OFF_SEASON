// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

/** Add your docs here. */
public class ShooterIntakeConstants {
    public enum PresetSpeeds{
        SPEED1(4000),
        SPEED2(3000),
        SPEED3(2000),
        SPEED4(1000);
    
        public final double RPM;
    
        private PresetSpeeds(double RPM){
          this.RPM = RPM;
        }
      }
      
}
