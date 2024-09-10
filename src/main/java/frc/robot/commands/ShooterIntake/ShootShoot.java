// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootShoot extends SequentialCommandGroup {
    private static final double INTAKE_SPEED = ShooterIntakeConstants.PresetSpeeds.SPEED4.RPM;
    ShooterIntake shooterIntake;
    private final double rpm;
  /** Creates a new ShootShoot. */ 
  public ShootShoot(double rpm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(shooterIntake);
    this.rpm = rpm;
    addCommands(new Step1(),
      new Step2(),
      new InstantCommand());
  }
  
  private class Step1 extends Command{
    Timer timer = new Timer();
    @Override
    public void initialize(){
      shooterIntake.setTargetRPMShooterMotorOffPivot(rpm);
      shooterIntake.setTargetRPMShooterMotorOnPivot(rpm);
    }
//TODO corret time
  @Override
  public boolean isFinished() {
    return (shooterIntake.isShooterMotorsAtSetSpeed()) || timer.get() >=2;
  }

  }

  private class Step2 extends Command{
    Timer timer = new Timer();
    private Step2(){
      
    }

    @Override
    public void initialize() {
      shooterIntake.setTargetIntakeMotorRPM(INTAKE_SPEED);

    }

    @Override
    public void end(boolean interrupted) {
      shooterIntake.setGpLoaded(false);
      shooterIntake.stopIntakeMotor();
      shooterIntake.stopMotorOffPivot();
      shooterIntake.StopMotorOnPivot();
    }
  @Override
  public boolean isFinished() {
    return timer.get() >=2;
    
  }

  }
}

