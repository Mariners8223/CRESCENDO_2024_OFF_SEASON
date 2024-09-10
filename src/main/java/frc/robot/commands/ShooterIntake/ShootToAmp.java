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
public class ShootToAmp extends SequentialCommandGroup {
      private static final double INTAKE_SPEED = ShooterIntakeConstants.PresetSpeeds.SPEED6.RPM;

      ShooterIntake shooterIntake;
      Timer timer = new Timer(); 

  /** Creates a new ShootToAmp. */
  public ShootToAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Step1(),
      new InstantCommand(),
      new ParallelRaceGroup(null));
    timer.start();
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
      shooterIntake.setTargetIntakeMotorRPM(0);
    }
  @Override
  public boolean isFinished() {
    return timer.get() <=5;

  }

  }


  }
