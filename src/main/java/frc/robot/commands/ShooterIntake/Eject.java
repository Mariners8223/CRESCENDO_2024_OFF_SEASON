// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

public class Eject extends Command {
  ShooterIntake shooterIntake;
  /** Creates a new Eject. */
  public Eject(ShooterIntake shooterIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.shooterIntake = shooterIntake;

    addRequirements(shooterIntake);
  }
  
  public static Command getCommand(ShooterIntake shooterIntake){
    return new Eject(shooterIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterIntake.setIntakeMotorDutyCycle(ShooterIntakeConstants.Intake_Speeds.EJECT_POWER.value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterIntake.stopIntakeMotor();
  }
}
