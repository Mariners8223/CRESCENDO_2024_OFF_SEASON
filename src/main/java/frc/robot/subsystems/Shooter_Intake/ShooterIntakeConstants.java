// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;
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

  public enum ShooterPresetSpeeds{
    
    ShooterSpeedHigh(2000),
    ShooterSpeedLow(1000),
    IntakeShooterSpeedHigh(-2000),
    IntakeShooterSpeedLow(-1000);

    public final double RPM;
    
    private ShooterPresetSpeeds(double RPM){
      this.RPM = RPM;
    }

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
}
