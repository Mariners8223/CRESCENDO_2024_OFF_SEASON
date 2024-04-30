// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIOCompBot;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIODevBot;
import frc.util.FastGyros.GyroIO;
import frc.util.FastGyros.NavxIO;
import frc.util.FastGyros.SimGyroIO;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.concurrent.locks.ReentrantLock;

/**
 * The DriveBase class represents the drivetrain of the robot.
 * It controls the movement and positioning of the robot using swerve drive.
 */
public class DriveBase extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4]; //the array of the modules

  private final SwerveDriveKinematics driveTrainKinematics = new SwerveDriveKinematics(Constants.DriveTrain.SwerveModule.moduleTranslations); //the kinematics of the swerve drivetrain

  private final SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}; //the deltas of the modules

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(driveTrainKinematics, new Rotation2d(), moduleDeltas, new Pose2d()); //the pose estimator of the drivetrain

  private final GyroIO gyro; //the navx gyro of the robot

  private final DriveBaseInputsAutoLogged inputs = new DriveBaseInputsAutoLogged(); //an object representing the logger class

  private final ReentrantLock odometryLock = new ReentrantLock(); //a lock for the odometry and modules thread

  private final double maxFreeWheelSpeed = Constants.robotType == Constants.RobotType.DEVELOPMENT ? SwerveModuleIODevBot.DevBotConstants.maxDriveVelocityMetersPerSecond : SwerveModuleIOCompBot.CompBotConstants.maxDriveVelocityMetersPerSecond; //the max speed the wheels can spin when the robot is not moving

  private SwerveModuleState[] targetStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

  private Pose2d currentPose = new Pose2d();


  @AutoLog
  public static class DriveBaseInputs{
    protected double XspeedInput = 0; //the X speed input
    protected double YspeedInput = 0; //the Y speed input

    protected double rotationSpeedInput = 0;

    protected SwerveModuleState[] currentStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()}; //the current states of the modules

    protected String activeCommand; //the active command of the robot
  }

  /** Creates a new DriveBase. */
  public DriveBase() {
    modules[0] = new SwerveModule(Constants.DriveTrain.front_left);
    modules[1] = new SwerveModule(Constants.DriveTrain.front_right);
    modules[2] = new SwerveModule(Constants.DriveTrain.back_left);
    modules[3] = new SwerveModule(Constants.DriveTrain.back_right);

    if(RobotBase.isReal()){
      gyro = new NavxIO();
      gyro.reset(new Pose2d());
    }
    else gyro = new SimGyroIO(() -> driveTrainKinematics.toTwist2d(moduleDeltas), this::getChassisSpeeds);

    for(int i = 0; i < 4; i++) modules[i].resetDriveEncoder();

    SmartDashboard.putData("Gyro", gyro);

    ReplanningConfig replanConfig = new ReplanningConfig(Constants.DriveTrain.PathPlanner.planPathToStartingPointIfNotAtIt, Constants.DriveTrain.PathPlanner.enableDynamicRePlanning, Constants.DriveTrain.PathPlanner.pathErrorTolerance, Constants.DriveTrain.PathPlanner.pathErrorSpikeTolerance);
    // ^how pathplanner reacts to position error
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      Constants.DriveTrain.PathPlanner.XYPID.createPIDConstants(),
      Constants.DriveTrain.PathPlanner.thetaPID.createPIDConstants(),
      maxFreeWheelSpeed,
      Math.sqrt(Math.pow((Constants.DriveTrain.Global.distanceBetweenWheels / 2), 2) * 2),
      replanConfig);
    //^creates path constraints for pathPlanner

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::reset,
      this::getChassisSpeeds,
      this::drive,
      pathFollowerConfig,
      () -> {if(DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
      else return false;},
      this);
    //^configures the auto builder for pathPlanner

    new Trigger(RobotState::isEnabled).whileTrue(new StartEndCommand(() -> // sets the modules to brake mode when the robot is enabled
      setModulesBrakeMode(true)
    , () ->
      {if(!DriverStation.isFMSAttached()) setModulesBrakeMode(false);}
    ).ignoringDisable(true));

    new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
    this.setDefaultCommand(new DriveCommand()), this::removeDefaultCommand).ignoringDisable(true));
    }

  public @NotNull Notifier getNotifier() {
    SwerveModulePosition[] previousPositions = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    SwerveModulePosition[] positions = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    Runnable odometryAndModulesRunnable = () -> {
      for (int i = 0; i < 4; i++) {
        positions[i] = modules[i].modulePeriodic();

        moduleDeltas[i] = new SwerveModulePosition(
          positions[i].distanceMeters - previousPositions[i].distanceMeters,
          positions[i].angle
        );

        previousPositions[i] = positions[i].copy();
      }

      // System.out.println(driveTrainKinematics.toTwist2d(moduleDeltas));
      // System.out.println(moduleDeltas[0]);
      gyro.update();

      try {
        odometryLock.lock();
        poseEstimator.updateWithTime(Logger.getTimestamp(), gyro.getRotation2d(), positions);
      } finally {
        odometryLock.unlock();
      }
    };

    return new Notifier(odometryAndModulesRunnable);
  }


  /**
   * resets the robot to 0, 0 and a rotation of 0 (towards red alliance)
   */
  public void resetOnlyDirection(){
    if(DriverStation.getAlliance().isPresent()) if(DriverStation.getAlliance().get() == Alliance.Blue) currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());
    else currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(-Math.PI));
    else currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());

    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

    try {
      odometryLock.lock();
      poseEstimator.resetPosition(new Rotation2d(), positions, currentPose);
    } finally {
      odometryLock.unlock();
    }

    gyro.reset(currentPose);
  }

  public void setModulesBrakeMode(boolean isBrake){
    for(int i = 0; i < 4; i++){
      modules[i].setIdleMode(isBrake);
    }
  }
  /**
   * resets the robot to a new pose
   * @param newPose the new pose the robot should be in
   */
  public void reset(Pose2d newPose){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

    try {
      odometryLock.lock();
      poseEstimator.resetPosition(new Rotation2d(), positions, newPose);
    } finally {
      odometryLock.unlock();
    }

    gyro.reset(newPose);
    currentPose = newPose;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * gets the acceleration of the robot in the X direction
   * @return the acceleration of the robot in the X direction m/s^2
   */
  public double getXAcceleration(){
    return gyro.getAccelerationX();
  }

  /**
   * gets the acceleration of the robot in the Y direction
   * @return the acceleration of the robot in the Y direction m/s^2
   */
  public double getYAcceleration() {
    return gyro.getAccelerationY();
  }
  /**
   * returns the reported Rotation by the Navx
   * @return Rotation2d reported by the Navx
   */
  public Rotation2d getRotation2d(){
    return gyro.getRotation2d();
  }

  /**
   * returns the current angle of the robot
   * @return the angle of the robot (left is positive) IN DEGREES
   */
  public double getAngle(){
    return gyro.getAngleDegrees();
  }

  /**
   * gets the current chassisSpeeds of the robot
   * @return the current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds(){
    return driveTrainKinematics.toChassisSpeeds(inputs.currentStates);
  }

  /**
   * gets the current absolute (field relative ) speeds of the robot
   * @return the current chassis speeds
   */
  public ChassisSpeeds getAbsoluteChassisSpeeds(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
  }

  /**
   * gets the current pose of the robot
   * @return the current pose of the robot
   */
  public Pose2d getPose(){
    return currentPose;
  }

  /**
   * updates pose Estimator with vision measurements
   * @param visionPose the pose of the robot from vision
   * @param timeStamp the time stamp of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionPose, double timeStamp){
    poseEstimator.addVisionMeasurement(visionPose, timeStamp);
  }

  /**
   * drives the robot with angle PID fix
   * @param Xspeed the speed in the X direction (positive is away from the driver station)
   * @param Yspeed the speed in the Y direction (positive is left)
   * @param rotation the change in the angle of the robot (Positive is to rotate left)
   * @param centerOfRotation the center that the robot rotates about
   */
  public void drive(double Xspeed, double Yspeed, double rotation, Translation2d centerOfRotation){

    targetStates = driveTrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotation, getRotation2d()), centerOfRotation); //calculates the target states
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxFreeWheelSpeed); //desaturates the wheel speeds (to make sure none of the wheel exceed the max speed)

    for(int i = 0; i < 4; i++){
      targetStates[i] = modules[i].run(targetStates[i]); //sets the module state
    }

    inputs.XspeedInput = Xspeed; //logs the X speed before PID
    inputs.YspeedInput = Yspeed; //logs the Y speed before PID
    inputs.rotationSpeedInput = rotation; //logs the rotation speed before PID
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot relative to itself
   * @param Xspeed the X speed of the robot (forward is positive) m/s
   * @param Yspeed the Y speed of the robot (left is positive) m/s
   * @param rotation  the rotation of the robot (left is positive) rad/s
   */
  public void robotRelativeDrive(double Xspeed, double Yspeed, double rotation){

    targetStates = driveTrainKinematics.toSwerveModuleStates(new ChassisSpeeds(Xspeed, Yspeed, rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(inputs.currentStates, maxFreeWheelSpeed);

    for(int i = 0; i < 4; i++){
      targetStates[i] = modules[i].run(targetStates[i]);
    }

    inputs.XspeedInput = Xspeed;
    inputs.YspeedInput = Yspeed;
    inputs.rotationSpeedInput = rotation;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot with angle PID fix
   * @param Xspeed the speed in the X direction (positive is away from the driver station)
   * @param Yspeed the speed in the Y direction (positive is left)
   * @param rotation the change in the angle of the robot (Positive is to rotate left)
   */
  public void drive(double Xspeed, double Yspeed, double rotation){drive(Xspeed, Yspeed, rotation, new Translation2d());}

  /**
   * drives the robot without built in pid fixes
   * @param chassisSpeeds the chassis speeds of the target
   */
  public void drive(ChassisSpeeds chassisSpeeds){
    targetStates = driveTrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxFreeWheelSpeed);

    for(int i = 0; i < 4; i++){
      targetStates[i] = modules[i].run(targetStates[i]);
    }

    inputs.XspeedInput = chassisSpeeds.vxMetersPerSecond;
    inputs.YspeedInput = chassisSpeeds.vyMetersPerSecond;
    inputs.rotationSpeedInput = chassisSpeeds.omegaRadiansPerSecond;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot without any PID control at all
   * @param Xspeed the speed in the X direction (positive is away from the driver station)
   * @param Yspeed the speed in the Y direction (positive is left)
   * @param rotation the change in the angle of the robot (Positive is to rotate left)
   * @param centerOfRotation the center that the robot rotates about
   */
  public void driveWithOutPID(double Xspeed, double Yspeed, double rotation, Translation2d centerOfRotation){
    targetStates = driveTrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotation, getRotation2d()), centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxFreeWheelSpeed);

    for(int i = 0; i < 4; i++){
      targetStates[i] = modules[i].run(targetStates[i]);
    }

    inputs.XspeedInput = Xspeed;
    inputs.YspeedInput = Yspeed;
    inputs.rotationSpeedInput = rotation;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot without any PID control at all
   * @param Xspeed the speed in the X direction (positive is away from the driver station)
   * @param Yspeed the speed in the Y direction (positive is left)
   * @param rotation the change in the angle of the robot (Positive is to rotate left)
   */
  public void driveWithOutPID(double Xspeed, double Yspeed, double rotation){ driveWithOutPID(Xspeed, Yspeed, rotation, new Translation2d());}


  /**
   * path finds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
   * @param rotationDelay the delay in meters before the robot starts rotation
   * @return a command that follows a path to the target pose
   */
  public Command findPath(Pose2d targetPose, double endVelocity ,double rotationDelay){
    return AutoBuilder.pathfindToPose(targetPose, Constants.DriveTrain.PathPlanner.pathConstraints, endVelocity, rotationDelay);
  }

  /**
   * path finds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
   * @return a command that follows a path to the target pose
   */
  public Command findPath(Pose2d targetPose, double endVelocity){
    return AutoBuilder.pathfindToPose(targetPose, Constants.DriveTrain.PathPlanner.pathConstraints, endVelocity);
  }

  /**
   * path finds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @return a command that follows a path to the target pose
   */
  public Command findPath(Pose2d targetPose){
    return AutoBuilder.pathfindToPose(targetPose, Constants.DriveTrain.PathPlanner.pathConstraints);
  }

  /**
   * path finds to a given path then follows that path
   * @param targetPath the path to path find to and follow
   * @return a command that path finds to a given path then follows that path
   */
  public Command pathFindToPathAndFollow(PathPlannerPath targetPath){
    return AutoBuilder.pathfindThenFollowPath(targetPath, Constants.DriveTrain.PathPlanner.pathConstraints);
  }

  /**
   * path finds to a given path then follows that path
   * @param targetPath the path to path find to and follow
   * @param rotationDelay the delay in meters in which the robot does not change it's holonomic angle
   * @return a command that path finds to a given path then follows that path
   */
  public Command pathFindToPathAndFollow(PathPlannerPath targetPath, double rotationDelay){
    return AutoBuilder.pathfindThenFollowPath(targetPath, Constants.DriveTrain.PathPlanner.pathConstraints, rotationDelay);
  }

  /**
   * this updates the states of the modules, call this function periodically
   */
  public void update(){
      for(int i = 0; i < 4; i++){
        inputs.currentStates[i] = modules[i].getCurrentState();
      }

    try {
      odometryLock.lock();
      currentPose = poseEstimator.getEstimatedPosition();
    } finally {
      odometryLock.unlock();
    }

    Logger.recordOutput("DriveBase/estimatedPose", currentPose);
    Logger.recordOutput("DriveBase/ChassisSpeeds", getChassisSpeeds());
    Logger.recordOutput("DriveBase/targetStates", targetStates);

    RobotContainer.field.setRobotPose(currentPose);

    inputs.activeCommand = this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None";

    Logger.processInputs(getName(), inputs);
  }

  @Override
  public void periodic() {
    update();
  }


  private static class DriveCommand extends Command{
    DriveBase driveBase;
    // CommandPS5Controller controller;
    CommandPS4Controller controller;

    public DriveCommand(){
      controller = RobotContainer.driveController;
      driveBase = RobotContainer.driveBase;

      addRequirements(driveBase);
    }

    @Override
    public void initialize() {
      driveBase.drive(0, 0, 0);
    }

    @Override
    public void execute() {
      // driveBase.drive(
      //   //this basically takes the inputs from the controller and firsts checks if it's not drift or a mistake by checking if it is above a certain value then it multiplies it by the R2 axis that the driver uses to control the speed of the robot
      //   (Math.abs(controller.getLeftY()) > 0.05 ? -controller.getLeftY() : 0) * (controller.getR2Axis() > 0.1 ? 1 + (Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec - 1) * (1 - controller.getR2Axis()) : 1),

      //   (Math.abs(controller.getLeftX()) > 0.05 ? controller.getLeftX() : 0) * (controller.getR2Axis() > 0.1 ? 1 + (Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec - 1) * (1 - controller.getR2Axis()) : 1),

      //   Math.abs(controller.getRightX()) > 0.05 ? controller.getRightX() : 0
      //   );
      driveBase.drive(
        //this basically takes the inputs from the controller and firsts checks if it's not drift or a mistake by checking if it is above a certain value then it multiplies it by the R2 axis that the driver uses to control the speed of the robot
        (Math.abs(controller.getLeftY()) > 0.05 ? -controller.getLeftY() : 0) * driveBase.maxFreeWheelSpeed,

        (Math.abs(controller.getLeftX()) > 0.05 ? controller.getLeftX() : 0) * -driveBase.maxFreeWheelSpeed,

        (Math.abs(controller.getRightX()) > 0.05 ? controller.getRightX() : 0) * -driveBase.maxFreeWheelSpeed * 4
        );
    }

    @Override
    public void end(boolean interrupted) {
      driveBase.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
      return RobotState.isDisabled();
    }
  }

  public static class SysID{
    public enum SysIDType{
      Steer,
      Drive,
      X,
      Y,
    }
    SysIdRoutine steerSysId;
    SysIdRoutine driveSysId;
    SysIdRoutine xSpeedSysId;
    SysIdRoutine ySpeedSYSId;

    public SysID(DriveBase driveBase){
      steerSysId = new SysIdRoutine(
              new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("SysIDSteerState", state.toString())
              ),
              new SysIdRoutine.Mechanism(
                      (voltage) -> {
                        for(int i = 0; i < 4; i++) driveBase.modules[i].runSysID(null, voltage);
                      },
                      null,
                      driveBase,
                      "steerSysId"
              ));

      driveSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
          null, null, null, (state) -> Logger.recordOutput("SysIDDriveState", state.toString())
        ),
              new SysIdRoutine.Mechanism(
                      (voltage) -> {
                        for(int i = 0; i < 4; i++){
                          driveBase.modules[i].runSysID(voltage, null);
                        }
                      },
                      null,
                      driveBase,
                      "driveSysId"
              ));

      xSpeedSysId = new SysIdRoutine(
        new SysIdRoutine.Config(
          Units.Volts.of(0.5).per(Units.Seconds), Units.Volts.of(3), null, (state) -> Logger.recordOutput("SysIDXYState", state.toString())
        ),
              new SysIdRoutine.Mechanism(
                      (voltage) -> driveBase.drive(voltage.in(Units.Volts), 0, 0),
                      (log) -> Logger.recordOutput("/SysID/velocityX", driveBase.getChassisSpeeds().vxMetersPerSecond),
                      driveBase,
                      "xySpeedSysId"
              ));

      ySpeedSYSId = new SysIdRoutine(
              new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("SysIDYState", state.toString())
              ),
              new SysIdRoutine.Mechanism(
                      (voltage) -> driveBase.drive(0, voltage.in(Units.Volts), 0),
                      (log) -> Logger.recordOutput("/SysID/velocityY", driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond),
                      driveBase,
                      "ySpeedSysId"
              )
      );
    }

    public Command getSysIDCommand(SysIDType type, boolean isDynamic, boolean isForward){
      return isDynamic ? switch (type) {
        case Steer -> steerSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Drive -> driveSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case X -> xSpeedSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Y -> ySpeedSYSId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
      }
      : switch (type) {
        case Steer -> steerSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Drive -> driveSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case X -> xSpeedSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Y -> ySpeedSYSId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
      };
    }
  }
}
