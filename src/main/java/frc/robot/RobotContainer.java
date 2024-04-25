// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.DriveTrain.DriveBase.SysID;
import frc.robot.subsystems.DriveTrain.DriveBase.SysID.SysIDType;

public class RobotContainer{
    public static DriveBase driveBase;
    // public static CommandPS5Controller driveController;
    public static CommandPS4Controller driveController;

    public static Field2d field;

    public RobotContainer()
    {
        // driveController = new CommandPS5Controller(0);
        driveController = new CommandPS4Controller(0);
        driveBase = new DriveBase();
        SysID drivebaseSysID = new SysID(driveBase);

        configureBindings(drivebaseSysID);

        field = new Field2d();

        SmartDashboard.putData(field);
    }
    
    
    private void configureBindings(SysID drivebaseSysID) {
        driveController.options().onTrue(new InstantCommand(driveBase::resetOnlyDirection));
        driveController.cross().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, true, true));
        driveController.square().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, false, true));
        driveController.triangle().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, true, false));
        driveController.circle().whileTrue(drivebaseSysID.getSysIDCommand(SysIDType.Drive, false, false));
    }
    
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
