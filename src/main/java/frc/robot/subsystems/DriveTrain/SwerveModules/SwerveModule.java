package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import frc.util.PIDFGains;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.robotType;

public class SwerveModule {

    /**
     * the name of the swerve modules by order
     */
    public enum ModuleName {
        Front_Left,
        Front_Right,
        Back_Left,
        Back_Right
    }

    public static final double moduleThreadHz = 50;
    public static final double distanceBetweenWheels = 0.576; // the distance between each wheel in meters
    public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(distanceBetweenWheels / 2, distanceBetweenWheels / 2),
            new Translation2d(distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, -distanceBetweenWheels / 2)};

    private final String moduleName;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private boolean runningCalibration = false;
    private boolean runningDriveCalibration = false;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(ModuleName name) {
        this.moduleName = name.toString();

        if (RobotBase.isReal()) {
            io = switch (robotType) {
                case DEVELOPMENT -> new SwerveModuleIODevBot(name);
                case COMPETITION -> new SwerveModuleIOCompBot(name);
                case REPLAY -> throw new IllegalArgumentException("Robot cannot be replay if it's real");
            };
        } else {
            io = robotType == Constants.RobotType.REPLAY ? new SwerveModuleIOSIM.SwerveModuleIOReplay() : new SwerveModuleIOSIM();

        }

    }

    public SwerveModulePosition modulePeriodic() {
        io.updateInputs(inputs);

        if (!runningCalibration) {
            targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
            targetState.speedMetersPerSecond *= Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

            io.setDriveMotorReference(targetState.speedMetersPerSecond);
            io.setSteerMotorReference(targetState.angle.getRotations());

            io.run();
        } else {
            if (runningDriveCalibration) {
                double driveKp = SmartDashboard.getNumber("drive kP", 0);
                double driveKi = SmartDashboard.getNumber("drive kI", 0);
                double driveKd = SmartDashboard.getNumber("drive kD", 0);
                double driveKf = SmartDashboard.getNumber("drive kF", 0);

                io.setDriveMotorPID(new PIDFGains(driveKp, driveKi, driveKd, driveKf, 0, 0));

                double driveReference = SmartDashboard.getNumber("drive setPoint", 0);

                io.setDriveMotorReference(driveReference);

                io.setSteerMotorReference(targetState.angle.getRotations());

            } else {
                PIDController controller = (PIDController) SmartDashboard.getData(moduleName + " steer Controller");

                io.setSteerMotorPID(new PIDFGains(controller.getP(), controller.getI(), controller.getD()));
                io.setSteerMotorReference(targetState.angle.getRotations());
                io.setDriveMotorReference(0);
            }

            io.run();
        }
        Logger.processInputs("SwerveModule/" + moduleName, inputs);

        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.currentState.angle);
    }

    public SwerveModuleState getCurrentState() {
        return inputs.currentState;
    }

    public SwerveModuleState run(SwerveModuleState targetState) {
        targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
        targetState.speedMetersPerSecond *= inputs.currentState.angle.minus(targetState.angle).getCos();

        this.targetState = targetState;

        return targetState;
    }

    public void runSteerCalibration() {
        runningCalibration = true;

        PIDController steerPIDController = new PIDController(0, 0, 0);

        SendableRegistry.setName(steerPIDController, moduleName + " steer Controller");

        SmartDashboard.putData(steerPIDController);
    }

    public void stopSteerCalibration() {
        runningCalibration = false;
    }

    public void runDriveCalibration() {
        runningDriveCalibration = true;
        runningCalibration = true;
    }

    public void stopDriveCalibration() {
        runningCalibration = false;
        runningDriveCalibration = false;
    }

    public void setIdleMode(boolean isBrakeMode) {
        io.setIdleMode(isBrakeMode);
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }
}
