// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import edu.wpi.first.wpilibj.DutyCycle;
import frc.util.PIDFGains;

/** Add your docs here. */
public class ShooterIntakeConstants {

  public static final double INTAKE_MOTOR_UNDER_LOAD_CURRENT = 15;
  public static final double SHOOTER_MOTOR_UNDER_LOAD_CURRENT = 15;
  public static final double SHOOTERSPEED = 10;


  //TODO change times
  public enum AccelarationTime{
    SHOOTSHOOTTIME(5),
    SHOOTAMPTIME(5),
    INTAKESHOOTERTIME(5),
    INTAKEINTAKETIME(5);

    public final int sec;

    private AccelarationTime(int sec){
      this.sec = sec;
    }
  }

  public enum DezzNuts{

    PivotShooterSpeed(),
    IntakeIntakeSpeed(),
    IntakeShooterSpeed(),
    OutIntakeSpeed();

    public final double Arpm;
    private DezzNuts (double Arpm)
    {
      this.Arpm =Arpm;
    }  
    
  }


  public enum ShooterPresetSpeeds{
    
    ShooterSpeedHigh(2000),
    ShooterSpeedLow(1000),
    IntakeShooterSpeedHigh(-2000),
    IntakeShooterSpeedLow(-1000);

    public final double RPM;
    
    private ShooterPresetSpeeds(double RPM){
      this.RPM = RPM;
    }

//IntakeShooterIntake speed should be 0.175 
  }
  public enum IntakePresetSpeeds{
    ShootAmpSpeed( -500),
    IntakeSpeedHigh(2000),
    IntakeSpeedLow(1000),
    IntakeGetOutSpeed( -100),
    IntakeShooterIntake( 500);

    public final double RPM;
    
    private IntakePresetSpeeds(double RPM){
      this.RPM = RPM;
    }
  }

  public class IO_CONSTNATS{ 
      public static final int INTAKE_MOTOR_ID = 0;
      public static final int ON_PIVOT_SHOOTER_MOTOR_ID = 1;
      public static final int OFF_PIVOT_SHOOTER_MOTOR_ID = 3;
      public static final int BEAM_BREAK_PORT = 0;

      public static final double INTAKE_MOTOR_GEAR_RATIO = 1;
      public static final double SHOOTER_MAX_RPM = 5700;
    
    
    
      public static final boolean BEAM_BREAK_INVERTED = false;
      public static final boolean INTAKE_MOTOR_INVERTED = false;
      public static final boolean ON_PIVOT_SHOOTER_MOTOR_INVERTED = false;
      public static final boolean OFF_PIVOT_SHOOTER_MOTOR_INVERTED = false;

//משהו משהו שאיל רוצה שנעשה
      public static final PIDFGains ON_PIVOT_SHOOTER_PID = new  PIDFGains(0, 0, 0,0);
      public static final PIDFGains OFF_PIVOT_SHOOTER_PID = new  PIDFGains(0, 0, 0,0);
      public static final PIDFGains INTAKE_MOTOR_PID = new  PIDFGains(0, 0, 0,0);
    
    
    }
  }
