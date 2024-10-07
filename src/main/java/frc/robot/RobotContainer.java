// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.Arm.AlphaAim;
import frc.robot.commands.Arm.BetaAim;
import frc.robot.commands.Arm.MoveArmToPosition;
import frc.robot.commands.Climb.HookAscend;
import frc.robot.commands.Climb.HookDescend;
import frc.robot.commands.Drive.Auto_IntakeCommand;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.ShooterIntake.*;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;
import frc.robot.subsystems.Vision.VisionConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Vision.Vision;

public class RobotContainer {

    public static DriveBase driveBase;
    public static ShooterIntake shooterIntake;
    public static Climb climb;
    public static Vision vision;
    public static Arm arm;


    public static CommandPS5Controller driveController;
    public static CommandPS5Controller armController;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        armController = new CommandPS5Controller(1);

        driveBase = new DriveBase();
        shooterIntake = new ShooterIntake();
        climb = new Climb();
        arm = new Arm();
        vision = new Vision(
                driveBase::getPose,

                driveBase::addVisionMeasurement,

                DriveBaseConstants.CHASSIS_HEIGHT);


        configureBindings();
        configNamedCommands();

        field = new Field2d();

        SmartDashboard.putData(field);

        configChooser();

        new Trigger(RobotState::isAutonomous).onTrue(new InstantCommand(() -> {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                Constants.SourceConstants.SOURCE_ANGLE = Constants.SourceConstants.Blue.SOURCE_ANGLE;
            } else {
                Constants.SourceConstants.SOURCE_ANGLE = Constants.SourceConstants.Red.SOURCE_ANGLE;
            }
        }).ignoringDisable(true));
    }

    private static double rpm = 2000;
    private static Optional<Rotation2d> angleToSpeaker = Optional.empty();

    private static void configureBindings() {

        DriveCommand driveCommand =
                new DriveCommand(driveBase, driveController,
                        DriveBaseConstants.thetaControllerGains.createPIDController());

        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
                driveBase.setDefaultCommand(driveCommand),
                driveBase::removeDefaultCommand).ignoringDisable(true));

        driveController.options().onTrue(driveBase.resetOnlyDirection());


        Supplier<Measure<Angle>> alphaTarget = () -> {
            Vision.VisionOutPuts speakerAngle =
                    vision.getAngleToSpeakerFront(-ArmConstants.ALPHA_DISTANCE_FROM_CENTER.getX(),
                            ArmConstants.ALPHA_DISTANCE_FROM_CENTER.getZ(), ShooterIntakeConstants.SPEED_MULTIPLIER);

            rpm = speakerAngle.getSpeed().in(Units.RPM);

            angleToSpeaker = Optional.of(speakerAngle.getYaw());

            return speakerAngle.getPitch();
        };

        configureDriveBindings(driveCommand);
        configureArmBindings(alphaTarget);
        configureIntakeShooterBindings();
        configureClimbBindings();
    }

    private static void configureDriveBindings(DriveCommand driveCommand) {
        Command speakerAngle = new InstantCommand(() -> driveCommand.setTargetAngle(angleToSpeaker)).repeatedly();
        driveController.cross().whileTrue(speakerAngle).onFalse(driveCommand.emptyTargetAngle());

        RepeatCommand chassisToAmpAngle = new InstantCommand(() -> {
            Rotation2d angle = Rotation2d.fromRadians(MathUtil.angleModulus(driveBase.getRotation2d().getRadians()));

            Rotation2d target = Constants.AmpConstants.AMP_ANGLE.minus(angle);

            driveCommand.setTargetAngle(Optional.of(target));
        }).repeatedly();

        RepeatCommand chassisToSourceAngle = new InstantCommand(() -> {
            Rotation2d angle = Rotation2d.fromRadians(MathUtil.angleModulus(driveBase.getRotation2d().getRadians()));

            Rotation2d target = Constants.SourceConstants.SOURCE_ANGLE.minus(angle);

            driveCommand.setTargetAngle(Optional.of(target));
        }).repeatedly();

        driveController.square().whileTrue(chassisToSourceAngle).onFalse(driveCommand.emptyTargetAngle());
        driveController.circle().whileTrue(chassisToAmpAngle).onFalse(driveCommand.emptyTargetAngle());

        BooleanSupplier intakeRunning = () -> shooterIntake.getCurrentCommand().getName().equals("IntakeFromIntake");

        driveController.R1().whileTrue(
                Auto_IntakeCommand.getCommand(driveBase, vision::getAngleToGP, driveCommand::setTargetAngle, shooterIntake::isGpLoaded, intakeRunning)
                        .onlyIf(() -> arm.getCurrentPos() == ArmPosition.COLLECT_FLOOR_POSITION
                                && !shooterIntake.isGpLoaded())
        ).onFalse(driveCommand.emptyTargetAngle());

        
    }

    private static void configureArmBindings(Supplier<Measure<Angle>> alphaTarget) {
        Command moveToHome = MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.HOME_POSITION);

        Command resetDriveAngle =
                new InstantCommand(() -> vision.setPipeline(VisionConstants.PipeLineID.THREE_DIMENSIONAL, VisionConstants.CameraLocation.FRONT_RIGHT))
                .andThen(new InstantCommand(() -> angleToSpeaker = Optional.empty()));

        armController.cross().onTrue(new InstantCommand(() -> vision.setPipeline(VisionConstants.PipeLineID.TWO_DIMENSIONAL, VisionConstants.CameraLocation.FRONT_RIGHT)));

        armController.cross().whileTrue(AlphaAim.getCommand(arm, alphaTarget)).whileFalse(moveToHome);

        armController.cross().onFalse(resetDriveAngle);

        armController.circle().whileTrue(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.COLLECT_FLOOR_POSITION))
                .whileFalse(moveToHome);

        armController.square().whileTrue(
                        MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.COLLECT_SOURCE_POSITION))
                .whileFalse(moveToHome);

        armController.square().onTrue(new InstantCommand(() -> rpm = 2000));

        armController.triangle().whileTrue(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.AMP_POSITION))
                .whileFalse(moveToHome);
    }

    private static void configureIntakeShooterBindings() {
        Command intakeFromIntake = IntakeFromIntake.getCommand(shooterIntake);
        Command intakeFromShooter = IntakeFromShooter.getCommand(shooterIntake);

        armController.R1().onTrue(new InstantCommand(() -> {
            if (intakeFromIntake.isScheduled()) {
                intakeFromIntake.cancel();
                return;
            }

            if (intakeFromShooter.isScheduled()) {
                intakeFromShooter.cancel();
                return;
            }

            ArmPosition currentPosition = arm.getCurrentPos();

            if (currentPosition == ArmPosition.COLLECT_FLOOR_POSITION) intakeFromIntake.schedule();
            else if (currentPosition == ArmPosition.COLLECT_SOURCE_POSITION) intakeFromShooter.schedule();
        })).debounce(0.1);

        armController.L1().onTrue(
                new Eject(shooterIntake).onlyIf(() -> arm.getCurrentPos() != ArmPosition.COLLECT_FLOOR_POSITION)
                        .andThen(ShootToAmp.getCommand(shooterIntake).onlyIf(() -> arm.getCurrentPos() == ArmPosition.AMP_POSITION))
        ); //Shoot to amp


        Command updateSpeedWhenMoved = UpdateSpeedWhenMoved.getCommand(shooterIntake, () -> rpm);

        new Trigger(() -> armController.getR2Axis() >= 0.95).whileTrue(
                updateSpeedWhenMoved
        ).onFalse(
                new InstantCommand(updateSpeedWhenMoved::cancel)
                        .alongWith(ShootShoot.getCommand(shooterIntake, () -> rpm)).withName("shoot")
        );
    }

    private static void configureClimbBindings(DriveCommand drive) {
        armController.povUp().whileTrue(new HookAscend(climb, driveBase::getPose, driveBase::getRotation2d, drive::setTargetAngle));
        armController.povDown().whileTrue(new HookDescend(climb));
    }

    public static Command getAutoCommand() {
        return autoChooser.get();
    }


    public static void configNamedCommands() {
        NamedCommands.registerCommand("Move Arm To Home",
                MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.HOME_POSITION));

        NamedCommands.registerCommand("Move Arm To Collect Floor",
                MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.COLLECT_FLOOR_POSITION));

        NamedCommands.registerCommand("Move Arm To SubWoofer Position",
                MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.SHOOT_SUBWOFFER_POSITION).andThen(new InstantCommand(() -> rpm = 2500)));

        NamedCommands.registerCommand("Shoot", ShootShoot.getCommand(shooterIntake,
                () -> rpm));

        NamedCommands.registerCommand("Intake", IntakeFromIntake.getCommand(shooterIntake));

        Supplier<Measure<Angle>> supplier = () -> {
            Vision.VisionOutPuts speakerAngle =
                    vision.getAngleToSpeakerFront(
                            ArmConstants.ALPHA_DISTANCE_FROM_CENTER.getX() + ArmConstants.DISTANCE_BETWEEN_PIVOTS_METERS,
                            ArmConstants.ALPHA_DISTANCE_FROM_CENTER.getZ(), ShooterIntakeConstants.SPEED_MULTIPLIER);

            rpm = speakerAngle.getSpeed().in(Units.RPM);

            return speakerAngle.getPitch();
        };

        NamedCommands.registerCommand("Beta Aim", BetaAim.getCommand(arm, supplier));
        NamedCommands.registerCommand("Collect", IntakeFromIntake.getCommand(shooterIntake));
        NamedCommands.registerCommand("Collect From Floor",
        new SequentialCommandGroup(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.COLLECT_FLOOR_POSITION),
                                    IntakeFromIntake.getCommand(shooterIntake)));
        
        NamedCommands.registerCommand("Shoot to Speaker", new SequentialCommandGroup(
            BetaAim.getCommand(arm, supplier),
            ShootShoot.getCommand(shooterIntake, () -> rpm))
            );
    }


    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand";

        @Override
        public boolean getAsBoolean() {
            if (autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try {
                return !Objects.equals(lastAutoName, currentAutoName);
            } finally {
                lastAutoName = currentAutoName;
            }

        }
    };

    private void configChooser() {
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

        autoChooser = new LoggedDashboardChooser<>("chooser");
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            autosOfAutos.add(auto);
        }

        autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop)
                .onTrue(new InstantCommand(() -> field.getObject("AutoPath").setPoses()).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate)
                .onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            boolean invert =
                    DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
                path = invert ? path.flipPath() : path;

                poses.addAll(path.getPathPoses());
            });
        } catch (RuntimeException ignored) {
        }

        field.getObject("AutoPath").setPoses(poses);
    }

}
