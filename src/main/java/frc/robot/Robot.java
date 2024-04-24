// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Twist2d;
import frc.util.LocalADStarAK;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    
    private RobotContainer robotContainer;
    
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        Pathfinding.setPathfinder(new LocalADStarAK());
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        Logger.registerURCL(URCL.startExternal(Constants.DriveTrain.sparkMaxNames));

        if(isReal()){
            Logger.addDataReceiver(new RLOGServer());
            Logger.addDataReceiver(new WPILOGWriter("/U/logs/AdvantageKit"));

            DataLogManager.start("U/logs/dataLogManager");
            SignalLogger.enableAutoLogging(true);
            SignalLogger.setPath("U/logs/signalLogger");

        }
        else{
            Logger.addDataReceiver(new RLOGServer());
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = robotContainer.getAutonomousCommand();
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
