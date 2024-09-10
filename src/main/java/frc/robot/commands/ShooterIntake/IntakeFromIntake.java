// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromIntake extends SequentialCommandGroup {
private static final double INTAKE_SPEED = ShooterIntakeConstants.IntakePresetSpeeds.IntakeSpeedHigh.RPM;
private static final double SLOW_INTAKE_SPEED = ShooterIntakeConstants.IntakePresetSpeeds.IntakeSpeedLow.RPM;
  ShooterIntake shooterIntake;
  /** Creates a new IntakeFromIntake. */
  public IntakeFromIntake(ShooterIntake shooterIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());\

    this.shooterIntake = shooterIntake;

    addRequirements(shooterIntake);

    addCommands(
      new Step1(),
      new Step2()
      );
  }

  private class Step1 extends Command{
    private Step1(){
    }

    @Override
    public void initialize() {
      shooterIntake.setTargetIntakeMotorRPM(INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {

    }
  @Override
  public boolean isFinished() {
    return shooterIntake.isIntakeMotorsUnderLoad();
  }

  }

  private class Step2 extends Command{
    private Step2(){
    }

    @Override
    public void initialize() {
       shooterIntake.setTargetIntakeMotorRPM(SLOW_INTAKE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
      shooterIntake.stopIntakeMotor();
      shooterIntake.setGpLoaded(true);
    }
  @Override
  public boolean isFinished() {
    return shooterIntake.getBeamBreakValue();
  }

  }
  
}
