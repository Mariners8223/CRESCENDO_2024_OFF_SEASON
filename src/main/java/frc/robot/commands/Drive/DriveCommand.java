package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveTrain.DriveBase;


public class DriveCommand extends Command {
    private final DriveBase driveBase;
    private final CommandPS5Controller controller;

    public DriveCommand(DriveBase driveBase, CommandPS5Controller controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(this.driveBase);
    }

    private double lerp(double p){
      return 1 + (driveBase.maxFreeWheelSpeed - 1) * p;
    }

    @Override
    public void initialize() {
       driveBase.drive(0, 0, 0);
    }


    @Override
    public void execute() {
       driveBase.drive(
               //this basically takes the inputs from the controller and firsts checks if it's not drift or a mistake by checking if it is above a certain value then it multiplies it by the R2 axis that the driver uses to control the speed of the robot
               (Math.abs(controller.getLeftY()) > 0.1 ? -controller.getLeftY() : 0) * lerp(1 - (0.5 + controller.getR2Axis() / 2)),

               (Math.abs(controller.getLeftX()) > 0.1 ? -controller.getLeftX() : 0) * lerp(1 - (0.5 + controller.getR2Axis() / 2)),

               (Math.abs(controller.getRightX()) > 0.1 ? -controller.getRightX() : 0) * lerp(1 - (0.5 + controller.getR2Axis() / 2))
       );
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
      driveBase.drive(0, 0, 0);
    }
}
