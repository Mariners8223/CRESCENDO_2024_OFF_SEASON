// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import frc.robot.subsystems.Shooter_Intake.ShooterIntakeConstants;

import java.util.Map;

/**
 * Add your docs here.
 */
public class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }

    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT; //the type of robot the code is running on

    public static final Map<Integer, String> SPARK_MAX_NAMES = Map.of(
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Front_Left.ordinal()][1], "Front_Left",
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Front_Right.ordinal()][1], "Front_Right",
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Back_Left.ordinal()][1], "Back_Left",
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Back_Right.ordinal()][1], "Back_Right",
            ArmConstants.AlphaConstants.MOTOR_ID, "alpha motor",
            ArmConstants.BetaConstants.MOTOR_ID, "beta motor",
            ShooterIntakeConstants.IO_CONSTNATS.ON_PIVOT_SHOOTER_MOTOR_ID, "on pivot shooter",
            ShooterIntakeConstants.IO_CONSTNATS.OFF_PIVOT_SHOOTER_MOTOR_ID, "off pivot shooter",
            ShooterIntakeConstants.IO_CONSTNATS.INTAKE_MOTOR_ID, "intake motor"
    );

    public static class AmpConstants {
        public static Rotation2d AMP_ANGLE = Rotation2d.fromRadians(Math.PI / 2);
    }

    public static class SourceConstants{
        public static Rotation2d SOURCE_ANGLE = new Rotation2d();

        public static class Red{
            public static Rotation2d SOURCE_ANGLE = Rotation2d.fromDegrees(-30 - 90);
        }

        public static class Blue{
            public static Rotation2d SOURCE_ANGLE = Rotation2d.fromDegrees(30 - 90);
        }
    }
}
