// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromIntake extends Command {
  ShooterIntake shooterIntake;
  boolean isEnded = false;
  Notifier thread;
  /** Creates a new IntakeFromIntake. */
  private IntakeFromIntake(ShooterIntake shooterIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());\

    this.shooterIntake = shooterIntake;

    thread = new Notifier(this::perdioc);

    addRequirements(shooterIntake);
  }

  public static Command getCommand(ShooterIntake shooterIntake){
    return new IntakeFromIntake(shooterIntake).onlyIf(() -> !shooterIntake.isGpLoaded());
  }

  @Override
  public void initialize() {
    isEnded = false;
    shooterIntake.setIntakeMotorDutyCycle(ShooterIntakeConstants.Intake_Speeds.INTAKE_POWER.value);
    thread.startPeriodic(0.01);
  }

  private void perdioc(){
    if(isEnded) return;

    if(shooterIntake.getBeamBreakValue()){
      isEnded = true;
      shooterIntake.setIntakeMotorTargetPosition(shooterIntake.getIntakeMotorPositions());
    }
  }

  @Override
  public void end(boolean interrupted) {
    isEnded = false;
    thread.stop();
    shooterIntake.setGpLoaded(!interrupted);
  }

  @Override
  public boolean isFinished() {
    return isEnded;
  }
  
}

