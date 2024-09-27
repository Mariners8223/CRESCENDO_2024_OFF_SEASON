// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterIntake.Eject;
import frc.robot.commands.ShooterIntake.IntakeFromIntake;
import frc.robot.commands.ShooterIntake.IntakeFromShooter;
import frc.robot.commands.ShooterIntake.ShootShoot;
import frc.robot.commands.ShooterIntake.ShootToAmp;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Shooter_Intake.ShooterIntake;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

public class RobotContainer{
    public static DriveBase driveBase;
    public static ShooterIntake shooterIntake;
    public static CommandPS5Controller driveController;
    public static CommandPS5Controller armController;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer()
    {
        driveController = new CommandPS5Controller(0);
        armController = new CommandPS5Controller(1);
        driveBase = new DriveBase();
        shooterIntake = new ShooterIntake();

        configureBindings();

        field = new Field2d();

        SmartDashboard.putData(field);

        configChooser();
    }

    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand"; 
        @Override
        public boolean getAsBoolean() {
            if(autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try{ 
                return !Objects.equals(lastAutoName, currentAutoName);
            }
            finally{
                lastAutoName = currentAutoName;
            }
            
        }
    };

    private void configChooser(){
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

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> field.getObject("AutoPath").setPoses()).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName){
        List<Pose2d> poses = new ArrayList<>();

        try {
            boolean invert =
                    DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
                path = invert ? path.flipPath() : path;

                poses.addAll(path.getPathPoses());
            });
        }
        catch (RuntimeException ignored){
        }

        field.getObject("AutoPath").setPoses(poses);
    }
    
    
    private void configureBindings() {
        driveController.options().onTrue(driveBase.resetOnlyDirection());
        // driveController.cross().onTrue(driveBase.runModuleDriveCalibration());
        // driveController.triangle().onTrue(driveBase.stopModuleDriveCalibration());


        SmartDashboard.putNumber("speed", 0);

        // armController.cross().whileTrue(new UpdateSpeedWhenMoved(shooterIntake, () -> SmartDashboard.getNumber("speed", 0)));
        armController.circle().onTrue(IntakeFromIntake.getCommand(shooterIntake));
        armController.R1().onTrue(ShootToAmp.getCommand(shooterIntake));
        armController.triangle().onTrue(ShootShoot.getCommand(shooterIntake, () -> ShooterIntakeConstants.Shooter_Speeds.SHOOT_SPEED.value));
        armController.cross().onTrue(IntakeFromShooter.getCommand(shooterIntake));
        armController.L1().whileTrue(Eject.getCommand(shooterIntake));
    }
    
    
    
    public static Command getAutoCommand()
    {
        return autoChooser.get();
    }
}
