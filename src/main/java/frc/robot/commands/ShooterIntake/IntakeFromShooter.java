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


  public static ConditionalCommand getCommand(ShooterIntake shooterIntake){
    ConditionalCommand command = new IntakeFromShooter(shooterIntake).onlyIf(() -> !shooterIntake.isGpLoaded());

    command.setName("Intake from Shooter");

    return command;
  }
  

  private class Step1 extends Command{
    @Override
    public void initialize() {
      //TODO maybe restart timer?
      shooterIntake.setTargetShooterMotorOffPivotDutyCycle(ShooterIntakeConstants.Shooter_Speeds.INTAKE_POWER.value);
      shooterIntake.setTargetShooterMotorOnPivotDutyCycle(ShooterIntakeConstants.Shooter_Speeds.INTAKE_POWER.value);
      shooterIntake.setIntakeMotorDutyCycle(ShooterIntakeConstants.Intake_Speeds.INTAKE_FROM_SHOOTER_POWER.value);
    }
    @Override
    public void end(boolean interrupted) {
      shooterIntake.stopMotorOffPivot();
      shooterIntake.StopMotorOnPivot();

    }

   @Override
   public boolean isFinished() {
    return shooterIntake.getBeamBreakValue();
    }
  }

  private class Step2 extends Command{
    Timer timer = new Timer();
    int counter;
    @Override
    public void initialize() {
      timer.restart();
      counter = 0;
    }

    @Override
    public void execute(){
      counter++;
    }

    @Override
    public void end(boolean interrupted) {
      shooterIntake.setGpLoaded(true);
      shooterIntake.setIntakeMotorTargetPosition(shooterIntake.getIntakeMotorPositions() - 0.98);

    }
   @Override
    public boolean isFinished() {
     return counter >= 8;
    }
  }
}
