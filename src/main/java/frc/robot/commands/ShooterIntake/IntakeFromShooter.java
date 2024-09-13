// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromShooter extends SequentialCommandGroup {
  ShooterIntake shooterIntake;
  /** Creates a new IntakeFromShooter. */
  public IntakeFromShooter(ShooterIntake shooterIntake) {

    this.shooterIntake = shooterIntake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addRequirements(shooterIntake);

    addCommands(
      new Step1(),
      new Step2()
    );
  }
  private class Step1 extends Command{
    @Override
    public void initialize() {
      shooterIntake.setTargetRPMShooterMotorOffPivot(ShooterIntakeConstants.ShooterPresetSpeeds.IntakeShooterSpeedHigh.RPM);
      shooterIntake.setTargetRPMShooterMotorOnPivot(ShooterIntakeConstants.ShooterPresetSpeeds.ShooterSpeedHigh.RPM);
    }

    @Override
    public void end(boolean interrupted) {
      shooterIntake.setTargetRPMShooterMotorOffPivot(ShooterIntakeConstants.ShooterPresetSpeeds.IntakeShooterSpeedLow.RPM);
      shooterIntake.setTargetRPMShooterMotorOnPivot(ShooterIntakeConstants.ShooterPresetSpeeds.ShooterSpeedLow.RPM);
    }
   @Override
   public boolean isFinished() {
    return shooterIntake.getBeamBreakValue();
    }
  }

  private class Step2 extends Command{
    Timer timer = new Timer();
    @Override
    public void initialize() {
      timer.restart();
      shooterIntake.setTargetIntakeMotorRPM(ShooterIntakeConstants.IntakePresetSpeeds.IntakeSpeedLow.RPM);
    }

    @Override
    public void end(boolean interrupted) {
      shooterIntake.stopMotorOffPivot();
      shooterIntake.stopIntakeMotor();
      shooterIntake.StopMotorOnPivot();
      shooterIntake.setGpLoaded(!interrupted);
    }
   @Override
    public boolean isFinished() {
     return  timer.get() >= ShooterIntakeConstants.AccelarationTime.INTAKESHOOTERTIME.sec && shooterIntake.getBeamBreakValue();
    }
  }
}
