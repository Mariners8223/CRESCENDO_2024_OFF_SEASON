// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateSpeedIfMoved extends SequentialCommandGroup {
  ShooterIntake shooterIntake;
  private double rpm;

  /** Creates a new UpdateSpeedIfMoved. */
  public UpdateSpeedIfMoved(ShooterIntake shooterIntake) {
    this.shooterIntake = shooterIntake;
    addRequirements(shooterIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Step1()
    );
  }
  private class Step1 extends Command{

    @Override
    public void execute () {
      shooterIntake.setTargetRPMShooterMotorOffPivot(rpm);
      shooterIntake.setTargetRPMShooterMotorOnPivot(rpm);
    }

   @Override
    public boolean isFinished() {
      return !shooterIntake.isGpLoaded();
    }

  }
  
}

