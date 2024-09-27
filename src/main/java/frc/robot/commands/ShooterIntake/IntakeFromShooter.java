// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromShooter extends Command {
  ShooterIntake shooterIntake;
  Notifier beamBreaker;
  int counter;

  /** Creates a new IntakeFromShooter. */
  public IntakeFromShooter(ShooterIntake shooterIntake) {
    this.shooterIntake = shooterIntake;
    addRequirements(shooterIntake);

    beamBreaker = new Notifier(this::periodic);
    counter = 0;
  }

  public static ConditionalCommand getCommand(ShooterIntake shooterIntake){
    ConditionalCommand command = new IntakeFromShooter(shooterIntake).onlyIf(() -> !shooterIntake.isGpLoaded());

    command.setName("Intake from Shooter");

    return command;
  }

  private void periodic(){
    if (counter == ShooterIntakeConstants.INTAKE_CYCLE_TIME + 2) return;
    
    if(shooterIntake.getBeamBreakValue() && counter == 0){
      shooterIntake.stopMotorOffPivot();
      shooterIntake.StopMotorOnPivot();

      counter++;
    }

    else if(counter <= ShooterIntakeConstants.INTAKE_CYCLE_TIME && counter != 0) counter++; 

    else if(counter == ShooterIntakeConstants.INTAKE_CYCLE_TIME + 1){
      shooterIntake.stopIntakeMotor();
      counter++;
    }
  }

  @Override
  public void initialize() {
    //TODO maybe restart timer?
    shooterIntake.setTargetShooterMotorOffPivotDutyCycle(ShooterIntakeConstants.Shooter_Speeds.INTAKE_POWER.value);
    shooterIntake.setTargetShooterMotorOnPivotDutyCycle(ShooterIntakeConstants.Shooter_Speeds.INTAKE_POWER.value);
    shooterIntake.setIntakeMotorDutyCycle(ShooterIntakeConstants.Intake_Speeds.INTAKE_FROM_SHOOTER_POWER.value);

    beamBreaker.startPeriodic(0.005);
  }


  @Override
  public void end(boolean interrupted) {
    shooterIntake.stopMotorOffPivot();
    shooterIntake.StopMotorOnPivot();
    shooterIntake.setIntakeMotorTargetPosition(shooterIntake.getIntakeMotorPositions());

    shooterIntake.setGpLoaded(true);
    beamBreaker.stop();
    counter = 0;
  }

  @Override
  public boolean isFinished() {
    return counter == ShooterIntakeConstants.INTAKE_CYCLE_TIME + 2;
  }
}
