// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootShoot extends SequentialCommandGroup {
  ShooterIntake shooterIntake;
  private double rpm;
  /** Creates a new ShootShoot. */ 
  public ShootShoot(double rpm, ShooterIntake shooterIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.shooterIntake = shooterIntake;
    addRequirements(shooterIntake);
    this.rpm = rpm;
    addCommands(
      new Step1(),
      new Step2()
    );
  }

  public ConditionalCommand getCommand(ShooterIntake shooterIntake){
    return new ShootShoot(rpm, shooterIntake).onlyIf(() -> shooterIntake.isGpLoaded());
  }
  
  private class Step1 extends Command{
    Timer timer = new Timer();
    @Override
    public void initialize(){
      timer.restart();
      shooterIntake.setTargetRPMShooterMotorOffPivot(rpm);
      shooterIntake.setTargetRPMShooterMotorOnPivot(rpm);
    }
  //TODO corret time
    @Override
    public boolean isFinished() {
      return shooterIntake.isShooterMotorsAtSetSpeed()
      || timer.get() >= ShooterIntakeConstants.AccelarationTime.SHOOTSHOOTTIME.sec;
    }

  }

  private class Step2 extends Command{
    Timer timer = new Timer();

    @Override
    public void initialize() {
      timer.restart();
      shooterIntake.setTargetIntakeMotorRPM(ShooterIntakeConstants.ShooterPresetSpeeds.IntakeShooterSpeedLow.RPM);
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
      return timer.get() >= ShooterIntakeConstants.AccelarationTime.SHOOTSHOOTTIME.sec;
    
  }

  }
}

