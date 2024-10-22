// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.commands.Arm.CalibrateLimitSwitch;
import frc.util.LocalADStarAK;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    
    @Override
    public void robotInit() {
        new RobotContainer();

        Logger.recordMetadata("Robot Type", Constants.ROBOT_TYPE.name());

        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            //noinspection DataFlowIssue
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            //noinspection DataFlowIssue
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        Logger.registerURCL(URCL.startExternal(Constants.SPARK_MAX_NAMES));

        if(isReal()){
            Logger.addDataReceiver(new WPILOGWriter("/U/logs/AdvantageKit"));
            // if(Constants.robotType == RobotType.DEVELOPMENT) Logger.addDataReceiver(new NT4Publisher());
//            Logger.addDataReceiver(new NT4Publisher());

            DataLogManager.start("U/logs/dataLogManager");
            SignalLogger.setPath("U/logs/signalLogger");
            SignalLogger.start();
        }
        else{
            if(Constants.ROBOT_TYPE == Constants.RobotType.REPLAY){
                String logPath = LogFileUtil.findReplayLog();

                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                Logger.setReplaySource(new WPILOGReader(logPath));
            }
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        Logger.recordOutput("Zero 2D", new Pose2d());
        Logger.recordOutput("Zero 3D", new Pose3d());

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((path) ->
                Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0])));

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
                Logger.recordOutput("PathPlanner/TargetPose", targetPose));

        PathfindingCommand.warmupCommand().schedule();

    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        // Vision.VisionOutPuts speakerAngle =
        //             RobotContainer.vision.getAngleToSpeakerFront(-ArmConstants.ALPHA_DISTANCE_FROM_CENTER.getX(),
        //                     ArmConstants.ALPHA_DISTANCE_FROM_CENTER.getZ(), ShooterIntakeConstants.SPEED_MULTIPLIER);

    }
    
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
    }
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = RobotContainer.getAutoCommand();

        CalibrateLimitSwitch.getCommand(RobotContainer.arm).schedule();
    }
    
    private boolean runnedCommand = false;
    
    @Override
    public void autonomousPeriodic() {
        if(runnedCommand) return;
        if(autonomousCommand != null && !autonomousCommand.isScheduled() && RobotContainer.arm.isCalibrated()){
            autonomousCommand.schedule();
            runnedCommand = true;
        }
    }
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void autonomousExit() {runnedCommand = false;}
    
    
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
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void testExit() {}
}
