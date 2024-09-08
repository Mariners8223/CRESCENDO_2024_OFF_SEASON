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
public class ShootShoot extends SequentialCommandGroup {
    private static final double INTAKE_SPEED = ShooterIntakeConstants.PresetSpeeds.SPEED4.RPM;
    
    ShooterIntake shooterIntake;
  /** Creates a new ShootShoot. */
  Timer timer = new Timer(); 
  public ShootShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
    timer.start();
  }
   private class Step1 extends Command{
    private Step1(){

    }

    @Override
    public void initialize() {
      shooterIntake.setTargetRPMShooterMotorOffPivot(INPUTED_SPEED);
      shooterIntake.setTargetRPMShooterMotorOnPivot(INPUTED_SPEED);
    }
    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
    }

  @Override
  public boolean isFinished() {
    if (shooterIntake.getTargetRPMShooterMotorOffPivot() && shooterIntake.getTargetRPMShooterMotorOnPivot() == (INPUTED_SPEED)){
        return true;
    }
       return false;
    
    //return shooterIntake.getTargetRPMShooterMotorOffPivot();
    //return shooterIntake.getTargetRPMShooterMotorOnPivot();

  }

  }

  private class Step2 extends Command{
    private Step2(){

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
    return timer.get() >=2;
  }

  }
}
