package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveTrain.DriveBase;

import static frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule.distanceBetweenWheels;


public class DriveCommand extends Command {

    private final DriveBase driveBase;
    private final CommandPS5Controller controller;

    private static final Translation2d[] possibleCenterOfRotations = {
            new Translation2d(distanceBetweenWheels / 2, 0),
            new Translation2d(distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
            new Translation2d(0, -distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels /2 , 0),
            new Translation2d( -distanceBetweenWheels, distanceBetweenWheels / 2),
            new Translation2d(0, distanceBetweenWheels / 2),
            new Translation2d(distanceBetweenWheels / 2, distanceBetweenWheels / 2)
    };

    public DriveCommand(DriveBase driveBase, CommandPS5Controller controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(this.driveBase);
        setName("DriveCommand");
    }

    @Override
    public void initialize() {
        driveBase.drive(0, 0, 0, new Translation2d());
    }

    private static double deadBand(double value) {
        return Math.abs(value) > 0.1 ? value : 0;
    }


    @Override
    public void execute() {
        //calculates a value from 1 to the max wheel speed based on the R2 axis
        double R2Axis = (1 - (0.5 + controller.getR2Axis() / 2)) * (driveBase.maxFreeWheelSpeed - 1) + 1;

        double povAngle = controller.getHID().getPOV();

        Translation2d centerOfRotation = povAngle == -1 ?
                new Translation2d() : possibleCenterOfRotations[(int)(povAngle / 45) % 8];

        //sets the value of the 3 axis we need (accounting for drift)
        double leftX = -deadBand(controller.getLeftX());
        double leftY = -deadBand(controller.getLeftY());
        double rightX = deadBand(controller.getRightX());

        //drives the robot with the values
        driveBase.drive(
                leftX * R2Axis,
                leftY * R2Axis,
                rightX * R2Axis,
                centerOfRotation
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0, new Translation2d());
    }
}
