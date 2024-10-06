package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.DriveTrain.DriveBase;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class Auto_IntakeCommand extends Command {
    private final DriveBase driveBase;

    private final Supplier<Optional<Measure<Angle>>> angleSupplier;

    private final Consumer<Optional<Rotation2d>> angleConsumer;

    private final BooleanSupplier gpLoaded;

    PIDController pidController = new PIDController(3, 0, 0);

    private Auto_IntakeCommand(DriveBase driveBase, Supplier<Optional<Measure<Angle>>> angleSupplier, BooleanSupplier gpLoaded, Consumer<Optional<Rotation2d>> angleConsumer) {
        this.driveBase = driveBase;
        this.angleSupplier = angleSupplier;
        this.angleConsumer = angleConsumer;
        this.gpLoaded = gpLoaded;

        pidController.setSetpoint(0);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveBase);
    }

    public static Command getCommand(DriveBase driveBase, Supplier<Optional<Measure<Angle>>> angleSupplier, Arm arm,
    BooleanSupplier gpLoaded, Consumer<Optional<Rotation2d>> angleConsumer, BooleanSupplier armInPosition, BooleanSupplier isIntakeRunning) {
        
        BooleanSupplier isPresent =
                () -> angleSupplier.get().isPresent() && arm.getCurrentPos() == ArmConstants.ArmPosition.COLLECT_FLOOR_POSITION
                && armInPosition.getAsBoolean() && isIntakeRunning.getAsBoolean();

        Command command = new Auto_IntakeCommand(driveBase, angleSupplier, gpLoaded, angleConsumer).onlyIf(isPresent);

        command.setName("Auto_IntakeCommand");

        return command;
    }

    @Override
    public void execute() {
        Optional<Measure<Angle>> angle = angleSupplier.get();

        if(angle.isEmpty()) return;

        double radToTarget = angle.get().in(Units.Radians);

        double xSpeed = 0;
        double ySpeed = 0;

        double theta = pidController.calculate(radToTarget);

        if(Math.abs(radToTarget) <= 0.35){
            xSpeed = Math.cos(theta) * 2;
            ySpeed = Math.sin(theta) * 2;
        }


        driveBase.robotRelativeDrive(
                xSpeed,
                ySpeed,
                theta
        );
    }

    @Override
    public boolean isFinished() {
        return gpLoaded.getAsBoolean();
    }

    private static final ChassisSpeeds chassisSpeeds =
            new ChassisSpeeds(0, 0, 0);
    @Override
    public void end(boolean interrupted) {
        driveBase.drive(chassisSpeeds);
        angleConsumer.accept(Optional.empty());
    }
}
