// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import frc.util.PIDFGains;

/** Add your docs here. */
public class ShooterIntakeConstants {

  public static final double INTAKE_MOTOR_UNDER_LOAD_CURRENT = 15;
  public static final double SHOOTER_MOTOR_UNDER_LOAD_CURRENT = 15;
  public static final double SHOOTER_SPEED_TOLRANCE = 100;


  //TODO change times
  public enum AccelarationTime{
    SHOOTSHOOTTIME(0.5),
    SHOOTAMPTIME(1),
    INTAKESHOOTERTIME(0.05),
    INTAKEINTAKETIME(5);

    public final double sec;

    private AccelarationTime(double sec){
      this.sec = sec;
    }
  }

  public enum Shooter_Speeds{
    SHOOT_SPEED(3000),
    INTAKE_POWER(-0.175);

    public final double value;

    Shooter_Speeds(double value){
      this.value = value;
    }
  }

  public enum Intake_Speeds{
    EJECT_SPEED(-0.12),
    SHOOT_POWER(0.5),
    INTAKE_SPEED(0.4),
    TEST(-0.2);


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

//משהו משהו שאיל רוצה שנעשה
      public static final PIDFGains ON_PIVOT_SHOOTER_PID = new  PIDFGains(0.0001, 0, 0, 0.00017, 0, 0, 0);
      public static final PIDFGains OFF_PIVOT_SHOOTER_PID = new  PIDFGains(0.00007, 0, 0,0.000165, 0, 0);
      public static final PIDFGains INTAKE_MOTOR_PID = new  PIDFGains(0.0002, 0, 0,0, 0, 0);
    
    
    }
  }
