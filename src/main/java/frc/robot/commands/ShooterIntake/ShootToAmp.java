// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootToAmp extends Command {
  ShooterIntake shooterIntake;
      

  /** Creates a new ShootToAmp. */
  
  public ShootToAmp(ShooterIntake shooterIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.shooterIntake = shooterIntake;

    addRequirements(shooterIntake);
  }

  public static Command getCommand(ShooterIntake shooterIntake){
    return new ShootToAmp(shooterIntake);
  }
  
  private Timer timer = new Timer(); 
  
  @Override
  public void initialize() {
    timer.restart();
    shooterIntake.setIntakeMotorDutyCycle(ShooterIntakeConstants.Intake_Speeds.AMP_POWER.value);
  }

  @Override
  public void end(boolean interrupted) {
    shooterIntake.stopIntakeMotor();
    //if(interrupted) shooterIntake.setGpLoaded(shooterIntake.isGpLoaded());
    shooterIntake.setGpLoaded(false);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= ShooterIntakeConstants.AccelarationTime.SHOOTAMPTIME.sec; 
  }

}

