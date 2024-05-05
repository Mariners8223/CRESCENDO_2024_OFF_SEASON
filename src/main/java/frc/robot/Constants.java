// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.MotorMap.DriveBase.Back_Left;
import frc.robot.MotorMap.DriveBase.Back_Right;
import frc.robot.MotorMap.DriveBase.Front_Left;
import frc.robot.MotorMap.DriveBase.Front_Right;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIOCompBot;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIODevBot;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIOCompBot.CompBotConstants;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIODevBot.DevBotConstants;
import frc.util.PIDFGains;

import java.util.Map;

/** Add your docs here. */
public class Constants {
    public enum RobotType{
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }

    public static final RobotType robotType = RobotType.COMPETITION; //the type of robot the code is running on


    public static final class DriveTrain{
        /**
         * the name of the swerve modules by order
         */
        public  enum ModuleName{
            Front_Left,
            Front_Right,
            Back_Left,
            Back_Right
        }

        public static final class PathPlanner{
            public static final boolean planPathToStartingPointIfNotAtIt = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean enableDynamicRePlanning = true; //if pathplanner should replan the path if the robot is beyond the tolerance or if the spike is too big
            public static final double pathErrorTolerance = 0.1; //the max error in position before pathPlanner replans the path in meters
            public static final double pathErrorSpikeTolerance = 1; //the max position spike before path planner replans the path in meters

            public static final PathConstraints pathConstraints = new PathConstraints(
                    robotType == RobotType.DEVELOPMENT ? SwerveModuleIODevBot.DevBotConstants.maxDriveVelocityMetersPerSecond : SwerveModuleIOCompBot.CompBotConstants.maxDriveVelocityMetersPerSecond,
                    10,
                    10,
                    20); //the constraints for pathPlanner

            public static final PIDFGains thetaPID = new PIDFGains(1.4574, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
            public static final PIDFGains XYPID = new PIDFGains(5.5, 0.055, 0.05); //the pid gains for the pid controller of the robot's velocity, units are meters per second
        }

        public static class SwerveModule{
            public static final double distanceBetweenWheels = 0.576; // the distance between each wheel in meters //TODO update based on new swerve maybe?
            public static final Translation2d[] moduleTranslations = new Translation2d[]
                {new Translation2d(distanceBetweenWheels / 2, distanceBetweenWheels / 2), new Translation2d(distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
                 new Translation2d(-distanceBetweenWheels / 2, distanceBetweenWheels / 2), new Translation2d(-distanceBetweenWheels / 2, -distanceBetweenWheels / 2)};
            //^ places the translation of each module in the array in order of the enum (front left, front right, back left, back right)

            public final ModuleName moduleName; //the name of the module (enum)

            public final int driveMotorID; // the CAN ID of the drive motor
            public final int steerMotorID; // the CAN ID of the steer motor
            public final int absEncoderID; // the CAN ID of the absolute encoder (this case CanCoder)

            public final boolean isSteerInverted; // if the steer motor output should be reversed
            public final boolean isDriveInverted; // if the drive motor output should be reversed
            public final boolean isAbsEncoderInverted; // if the reading of the absolute encoder should be inverted

            public final double absoluteEncoderZeroOffset; //the offset between the magnets zero and the modules zero in degrees

            public SwerveModule(ModuleName moduleName, int driveMotorID, int steerMotorID, int AbsEncoderID, double absoluteEncoderZeroOffset, boolean isSteerInverted, boolean isDriveInverted, boolean isAbsEncoderInverted){
                this.moduleName = moduleName;

                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.absEncoderID = AbsEncoderID;

                this.isDriveInverted = isDriveInverted;
                this.isSteerInverted = isSteerInverted;
                this.isAbsEncoderInverted = isAbsEncoderInverted;

                this.absoluteEncoderZeroOffset = absoluteEncoderZeroOffset;
            }
        }

        public static SwerveModule[] createConstants(){
            SwerveModule[] modules = new SwerveModule[4];

            boolean drive_inverted, steer_inverted, absEncoder_inverted;
            double front_left_zeroOffset, front_right_zeroOffset, back_left_zeroOffset, back_right_zeroOffset = 0;
            if(robotType == RobotType.DEVELOPMENT){
                drive_inverted = DevBotConstants.isDriveInverted;
                steer_inverted = DevBotConstants.isSteerInverted;
                absEncoder_inverted = DevBotConstants.isAbsEncoderInverted;

                front_left_zeroOffset = DevBotConstants.back_left_zeroOffset;
                front_right_zeroOffset = DevBotConstants.front_right_zeroOffset;
                back_left_zeroOffset = DevBotConstants.back_left_zeroOffset;
                back_right_zeroOffset = DevBotConstants.back_right_zeroOffset;
            }
            else{
                drive_inverted = CompBotConstants.isDriveInverted;
                steer_inverted = CompBotConstants.isSteerInverted;
                absEncoder_inverted = DevBotConstants.isAbsEncoderInverted;

                front_left_zeroOffset = CompBotConstants.back_left_zeroOffset;
                front_right_zeroOffset = CompBotConstants.front_right_zeroOffset;
                back_left_zeroOffset = CompBotConstants.back_left_zeroOffset;
                back_right_zeroOffset = CompBotConstants.back_right_zeroOffset;
            }

            modules[0] = new SwerveModule(ModuleName.Front_Left, Front_Left.driveMotor, Front_Left.steerMotor, Front_Left.absEncoder,
            front_left_zeroOffset, steer_inverted, drive_inverted, absEncoder_inverted);

            modules[1] = new SwerveModule(ModuleName.Front_Right, Front_Right.driveMotor, Front_Right.steerMotor, Front_Right.absEncoder,
            front_right_zeroOffset, steer_inverted, drive_inverted, absEncoder_inverted);

            modules[2] = new SwerveModule(ModuleName.Back_Left, Back_Left.driveMotor, Back_Left.steerMotor, Back_Left.absEncoder,
            back_left_zeroOffset, steer_inverted, drive_inverted, absEncoder_inverted);

            modules[3] = new SwerveModule(ModuleName.Back_Right, Back_Right.driveMotor, Back_Right.steerMotor, Back_Right.absEncoder,
            back_right_zeroOffset, steer_inverted, drive_inverted, absEncoder_inverted);

            return modules;
        }

        public static final Map<Integer, String> sparkMaxNames = Map.of(
            Front_Left.steerMotor, "Front_Left",
            Front_Right.steerMotor, "Front_Right",
            Back_Left.steerMotor, "Back_Left",
            Back_Right.steerMotor, "Back_Right"
        );
        
    } 
}
