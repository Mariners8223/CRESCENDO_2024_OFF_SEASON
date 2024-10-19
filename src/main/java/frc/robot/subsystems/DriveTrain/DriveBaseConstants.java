package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.path.PathConstraints;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {
        public static final class PathPlanner{
            public static final boolean PLAN_PATH_TO_STARTING_POINT = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean DYNAMIC_RE_PLANNING = true; //if pathplanner should replan the path if the robot is beyond the tolerance or if the spike is too big
            public static final double PATH_ERROR_TOLERANCE = 0.1; //the max error in position before pathPlanner replans the path in meters
            public static final double PATH_ERROR_SPIKE_TOLERANCE = 0.5; //the max position spike before path planner replans the path in meters

            public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                    Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ?
                            SwerveModuleConstants.DEVBOT.MAX_WHEEL_LINEAR_VELOCITY :
                            SwerveModuleConstants.COMPBOT.MAX_WHEEL_LINEAR_VELOCITY,
                    0.1,
                    10,
                    20); //the constraints for pathPlanner

            public static final PIDFGains THETA_PID = new PIDFGains(3, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
            public static final PIDFGains XY_PID = new PIDFGains(4, 0, 0.2); //the pid gains for the pid controller of the robot's velocity, units are meters per second
        }

        public static final double CHASSIS_HEIGHT = 0.15;

        public static final PIDFGains thetaControllerGains = new PIDFGains(6, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
}
