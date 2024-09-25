// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromIntake extends Command {
  ShooterIntake shooterIntake;
  /** Creates a new IntakeFromIntake. */
  private IntakeFromIntake(ShooterIntake shooterIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());\

    this.shooterIntake = shooterIntake;

    addRequirements(shooterIntake);
  }

  public static ConditionalCommand getCommand(ShooterIntake shooterIntake){
    return new IntakeFromIntake(shooterIntake).onlyIf(() -> !shooterIntake.isGpLoaded());
  }

  @Override
  public void initialize() {
    shooterIntake.setIntakeMotorDutyCycle(ShooterIntakeConstants.Intake_Speeds.INTAKE_SPEED.value);
  }

  @Override
  public void end(boolean interrupted) {
    shooterIntake.setIntakeMotorTargetPosition(shooterIntake.getIntakeMotorPositions() - 1);
    shooterIntake.setGpLoaded(!interrupted);
  }

  @Override
  public boolean isFinished() {
    return shooterIntake.getBeamBreakValue();
  }
  
}

