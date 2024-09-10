// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import edu.wpi.first.units.Time;

/** Add your docs here. */
public class ShooterIntakeConstants {
  //TODO change times
    public enum AccelarationTime{
        SHOOTSHOOTTIME(5),
        SHOOTAMPTIME(5),
        INTAKESHOOTERTIME(5),
        INTAKEINTAKETIME(5);

        public final int sec;

        private AccelarationTime(int sec){
          this.sec = sec;
        
        }}
       public static final double INTAKE_MOTOR_UNDER_LOAD_CURRENT = 15;
       public static final double SHOOTER_MOTOR_UNDER_LOAD_CURRENT = 15;
       public static final double SHOOTERSPEED = 10;

    public enum PresetSpeeds{
        SPEED1(4000),
        SPEED2(-3000),
        SPEED3(2000),
        SPEED4(1000),
        SPEED5(500),
        SPEED6(-500);
    
    
        public final double RPM;
    
        private PresetSpeeds(double RPM){
          this.RPM = RPM;
        }
      
    public enum PresetsTimeSec
    {
        TimeShooter(2);

        public final double Sec;

        private PresetsTimeSec (double Sec)
        {
            this.Sec=Sec;
        }
    


      
    }

    
  }
}

    

 
