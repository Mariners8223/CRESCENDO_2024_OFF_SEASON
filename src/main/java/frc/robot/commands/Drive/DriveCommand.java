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
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveBase);
    }

   private double lerp(double p){
      return 1 + (driveBase.maxFreeWheelSpeed - 1) * p;
   }

   /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
       driveBase.drive(0, 0, 0);
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
       driveBase.drive(
               //this basically takes the inputs from the controller and firsts checks if it's not drift or a mistake by checking if it is above a certain value then it multiplies it by the R2 axis that the driver uses to control the speed of the robot
               (Math.abs(controller.getLeftY()) > 0.1 ? -controller.getLeftY() : 0) * lerp(1 - (0.5 + controller.getR2Axis() / 2)),

               (Math.abs(controller.getLeftX()) > 0.1 ? -controller.getLeftX() : 0) * lerp(1 - (0.5 + controller.getR2Axis() / 2)),

               (Math.abs(controller.getRightX()) > 0.1 ? -controller.getRightX() : 0) * lerp(1 - (0.5 + controller.getR2Axis() / 2))
       );
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
      driveBase.drive(0, 0, 0);
    }
}
