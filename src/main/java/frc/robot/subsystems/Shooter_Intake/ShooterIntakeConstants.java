// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import frc.util.PIDFGains;

/** Add your docs here. */
public class ShooterIntakeConstants {

  public static final double INTAKE_MOTOR_UNDER_LOAD_CURRENT = 20;
  public static final double SHOOTER_MOTOR_UNDER_LOAD_CURRENT = 15;
  public static final double SHOOTER_SPEED_TOLRANCE = 100;
  public static final int INTAKE_CYCLE_TIME = 20;
  public static final double SPEED_MULTIPLIER = 1400;


  public enum AccelarationTime{
    SHOOTSHOOTTIME(2),
    SHOOTAMPTIME(0.6),
    INTAKESHOOTERTIME(0.05),
    INTAKEINTAKETIME(5);

    public final double sec;

    private AccelarationTime(double sec){
      this.sec = sec;
    }
  }

  public enum Shooter_Speeds{
    SHOOT_SPEED(800),
    INTAKE_POWER(-0.25);

    public final double value;

    Shooter_Speeds(double value){
      this.value = value;
    }
  }

  public enum Intake_Speeds{
    AMP_POWER(-0.2),
    EJECT_POWER(-0.1),
    SHOOT_POWER(0.5),
    INTAKE_POWER(0.4),
    INTAKE_FROM_SHOOTER_POWER(-0.3);


    public final double value;

    Intake_Speeds(double value){
      this.value = value;
    }
  }



  public class IO_CONSTNATS{ 
      public static final int INTAKE_MOTOR_ID = 16;
      public static final int ON_PIVOT_SHOOTER_MOTOR_ID = 14;
      public static final int OFF_PIVOT_SHOOTER_MOTOR_ID = 15;
      public static final int BEAM_BREAK_PORT = 4;

      public static final double INTAKE_MOTOR_GEAR_RATIO = 3;
      public static final double SHOOTER_MAX_RPM = 5800;
    
    
    
      public static final boolean BEAM_BREAK_INVERTED = false;
      public static final boolean INTAKE_MOTOR_INVERTED = true;
      public static final boolean ON_PIVOT_SHOOTER_MOTOR_INVERTED = false;
      public static final boolean OFF_PIVOT_SHOOTER_MOTOR_INVERTED = true;

      public static final PIDFGains ON_PIVOT_SHOOTER_PID = new  PIDFGains(0.0001, 0, 0, 0.00015, 0, 0, 0);
      public static final PIDFGains OFF_PIVOT_SHOOTER_PID = new  PIDFGains(0.00007, 0, 0,0.00015, 0, 0);
      public static final PIDFGains INTAKE_MOTOR_PID = new  PIDFGains(0.1, 0, 0,0, 0, 0);
    
    
    }
  }
