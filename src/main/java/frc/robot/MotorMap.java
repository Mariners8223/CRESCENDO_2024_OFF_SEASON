package frc.robot;

import frc.robot.Constants.RobotType;

public class MotorMap {
    public static class DriveBase{
        public static class Front_Left{
            public static int driveMotor = Constants.robotType == RobotType.DEVELOPMENT ? 2 : 2;
            public static int steerMotor = Constants.robotType == RobotType.DEVELOPMENT ? 3 : 3;
            public static int absEncoder = Constants.robotType == RobotType.DEVELOPMENT ? 10 : 3;
        }

        public static class Front_Right{
            public static int driveMotor = Constants.robotType == RobotType.DEVELOPMENT ? 4 : 4;
            public static int steerMotor = Constants.robotType == RobotType.DEVELOPMENT ? 5 : 5;
            public static int absEncoder = Constants.robotType == RobotType.DEVELOPMENT ? 11 : 0;
        }

        public static class Back_Left{
            public static int driveMotor = Constants.robotType == RobotType.DEVELOPMENT ? 6 : 6;
            public static int steerMotor = Constants.robotType == RobotType.DEVELOPMENT ? 7 : 7;
            public static int absEncoder = Constants.robotType == RobotType.DEVELOPMENT ? 12 : 1;
        }

        public static class Back_Right{
            public static int driveMotor = Constants.robotType == RobotType.DEVELOPMENT ? 8 : 8;
            public static int steerMotor = Constants.robotType == RobotType.DEVELOPMENT ? 9 : 9;
            public static int absEncoder = Constants.robotType == RobotType.DEVELOPMENT ? 13 : 2;
        }
    }
}
