// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants;

public class HookAscend extends Command {
  /** Creates a new HookAscend. */
  Climb climb;
  Supplier<Pose2d> poseSupplier;
  Supplier<Rotation2d> angleSupplier;
  Consumer<Optional<Rotation2d>> angleConsumer;

  double x;
  double y; 

  public HookAscend(Climb climb, Supplier<Pose2d> poseSupply, Supplier<Rotation2d> angleSupply, Consumer<Optional<Rotation2d>> angleConsumer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    addRequirements(climb);

    this.poseSupplier = poseSupply;
    this.angleSupplier = angleSupply;
    this.angleConsumer = angleConsumer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.startMotor(ClimbConstants.CLIMB_DESCEND_MOTOR_POWER);

    Pose2d pos = poseSupplier.get();
    x = pos.getX();
    y = pos.getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double robotAngle = Units.radiansToDegrees(MathUtil.angleModulus(angleSupplier.get().getRadians()));
    double targetAngle;

    DriverStation.Alliance alliance = DriverStation.getAlliance().get();

    switch (alliance){
      case Blue:
        if (x > ClimbConstants.StageConstants.BLUE_EDGE) targetAngle = 180;
        else if (y < ClimbConstants.StageConstants.MID_LINE_Y) targetAngle = 60;
        else if (y > ClimbConstants.StageConstants.MID_LINE_Y) targetAngle = -60;
        else{
          angleConsumer.accept(Optional.empty());
          break;
        }

        targetAngle = targetAngle - robotAngle;
        angleConsumer.accept(Optional.of(Rotation2d.fromDegrees(targetAngle)));
        break;
      
      case Red:
        if (x < ClimbConstants.StageConstants.RED_EDGE) targetAngle = 0;
        else if (y < ClimbConstants.StageConstants.MID_LINE_Y) targetAngle = 120;
        else if (y > ClimbConstants.StageConstants.MID_LINE_Y) targetAngle = -120;
        else{
          angleConsumer.accept(Optional.empty());
          break;
        }

        targetAngle = targetAngle - robotAngle;
        angleConsumer.accept(Optional.of(Rotation2d.fromDegrees(targetAngle)));
        break;
      
      default:
        angleConsumer.accept(Optional.empty());
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
