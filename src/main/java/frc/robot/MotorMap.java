package frc.robot;

import frc.robot.Constants.RobotType;

public class MotorMap {
    public static class DriveBase{
        public static int[][] MODULES = new int[][]{
                //Front Left
            {2, 3, Constants.robotType == RobotType.DEVELOPMENT ? 10 : 3}, //drive, steer, absEncoder
                //Front Right
            {4, 5, Constants.robotType == RobotType.DEVELOPMENT ? 11 : 0}, //drive, steer, absEncoder
                //Back Left
            {6, 7, Constants.robotType == RobotType.DEVELOPMENT ? 12 : 1}, //drive, steer, absEncoder
                //Back Right
            {8, 9, Constants.robotType == RobotType.DEVELOPMENT ? 13 : 2} //drive, steer, absEncoder
        };
    }
}
