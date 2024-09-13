// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;

public class UpdateSpeedWhenMoved extends Command {
  ShooterIntake shooterIntake;
  private Supplier<Double> rpmSupplier;
  /** Creates a new UpdateSpeedWhenMoved. */
  public UpdateSpeedWhenMoved(ShooterIntake shooterIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterIntake = shooterIntake;
    addRequirements(shooterIntake);
  }

  @Override
  public void execute () {
    double rpm = rpmSupplier.get();

    shooterIntake.setTargetRPMShooterMotorOffPivot(rpm);
    shooterIntake.setTargetRPMShooterMotorOnPivot(rpm);
  }
  
  public void end () {
    shooterIntake.StopMotorOnPivot();
    shooterIntake.stopMotorOffPivot();
  }

 @Override
  public boolean isFinished() {
    return !shooterIntake.isGpLoaded();
  }
}
