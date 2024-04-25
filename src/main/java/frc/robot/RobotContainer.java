// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBase.SysID;
import frc.robot.subsystems.DriveTrain.DriveBase.SysID.SysIDType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer{
    public static DriveBase driveBase;
    // public static CommandPS5Controller driveController;
    public static CommandPS4Controller driveController;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer()
    {
        // driveController = new CommandPS5Controller(0);
        driveController = new CommandPS4Controller(0);
        driveBase = new DriveBase();
        SysID drivebaseSysID = new SysID(driveBase);

        configureBindings(drivebaseSysID);

        field = new Field2d();

        SmartDashboard.putData(field);

        configChooser();

        autoChooser.addOption("tesatast", driveBase.findPath(new Pose2d(14 , 6, new Rotation2d())));
    }

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
        // autoChooser.addOption("Shoot Note", new ShootNote());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> field.getObject("AutoPath").setPoses()).ignoringDisable(true));
    }


    public static void updateFieldFromAuto(String autoName){
        List<Pose2d> poses = new ArrayList<>();
        boolean DoesExsit = false;
        for (String name : AutoBuilder.getAllAutoNames()) {
            if(name.equals(autoName)) DoesExsit = true;
        }
        if(!DoesExsit){
            field.getObject("AutoPath").setPoses();
            return;
        }
        PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) path = path.flipPath();
            path.getPathPoses().forEach(pose -> poses.add(pose));
        });
        field.getObject("AutoPath").setPoses(poses);
    }
    
    
    private void configureBindings(SysID drivebaseSysID) {
        driveController.options().onTrue(new InstantCommand(driveBase::resetOnlyDirection));
        driveController.cross().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, true, true));
        driveController.square().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, false, true));
        driveController.triangle().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, true, false));
        driveController.circle().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, false, false));
    }
    
    
    
    public static Command getAutoCommand()
    {
        return autoChooser.get();
    }
}
