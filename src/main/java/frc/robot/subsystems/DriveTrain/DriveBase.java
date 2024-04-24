// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIO;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleREAL;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleSIM;
import frc.util.FastGyros.FastGyro;
import frc.util.FastGyros.FastNavx;
import frc.util.FastGyros.FastSimGyro;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.concurrent.locks.ReentrantLock;

/**
 * The DriveBase class represents the drivetrain of the robot.
 * It controls the movement and positioning of the robot using swerve drive.
 */
public class DriveBase extends SubsystemBase {
  private final SwerveModuleIO[] modules = new SwerveModuleIO[4]; //the array of the modules

  private final SwerveDriveKinematics driveTrainKinematics = new SwerveDriveKinematics(Constants.DriveTrain.SwerveModule.moduleTranslations); //the kinematics of the swerve drivetrain

  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(driveTrainKinematics, new Rotation2d(),
          new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
          new Pose2d()); //the pose estimator of the drivetrain

  private final FastGyro gyro; //the navx gyro of the robot

  private final PIDController thetaCorrectionController; //the pid controller that fixes the angle of the robot

  private final PathConstraints pathConstraints; //the constraints for pathPlanner

  private final DriveBaseInputsAutoLogged inputs; //an object representing the logger class

  private final ReentrantLock odometryLock = new ReentrantLock(); //a lock for the odometry and modules thread

  private final SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}; //the deltas of the modules

  @AutoLog
  public static class DriveBaseInputs{
    protected double XspeedInput = 0; //the X speed input
    protected double YspeedInput = 0; //the Y speed input

    protected double rotationSpeedInputBeforePID = 0; //the rotation speed input before PID
    protected double rotationSpeedInputAfterPID = 0; //the rotation speed input after PID

    protected Pose2d currentPose = new Pose2d(); //the current pose of the robot
    protected Rotation2d targetRotation = new Rotation2d(); //the target rotation of the robot (in radians)

    protected SwerveModuleState[] currentStates = new SwerveModuleState[4]; //the current states of the modules
    protected SwerveModuleState[] targetStates = new SwerveModuleState[4]; //the target states of the modules

    protected String activeCommand; //the active command of the robot

    protected boolean isControlled;
  }

  /** Creates a new DriveBase. */
  public DriveBase() {
    if(RobotBase.isReal()){
      modules[0] = new SwerveModuleREAL(Constants.DriveTrain.front_left); //the front left module
      modules[1] = new SwerveModuleREAL(Constants.DriveTrain.front_right); //the front right module
      modules[2] = new SwerveModuleREAL(Constants.DriveTrain.back_left); //the back left module
      modules[3] = new SwerveModuleREAL(Constants.DriveTrain.back_right); //the back right module
    }
    else{
      modules[0] = new SwerveModuleSIM(Constants.DriveTrain.front_left); //the front left module
      modules[1] = new SwerveModuleSIM(Constants.DriveTrain.front_right); //the front right module
      modules[2] = new SwerveModuleSIM(Constants.DriveTrain.back_left); //the back left module
      modules[3] = new SwerveModuleSIM(Constants.DriveTrain.back_right); //the back right module
    }

    for(int i = 0; i < 4; i++) modules[i].resetDriveEncoder();


//    gyro = new FastNavx(); //creates a new navx gyro
    gyro = new FastSimGyro(() -> driveTrainKinematics.toTwist2d(moduleDeltas)); //creates a new sim gyro (for simulation

    // gyro.reset(new Pose2d());

    SmartDashboard.putData("Navx", gyro);

    thetaCorrectionController = Constants.DriveTrain.Global.thetaCorrectionPID.createPIDController(); //creates the pid controller of the robots angle

    ReplanningConfig replanConfig = new ReplanningConfig(Constants.DriveTrain.PathPlanner.planPathToStartingPointIfNotAtIt, Constants.DriveTrain.PathPlanner.enableDynamicRePlanning, Constants.DriveTrain.PathPlanner.pathErrorTolerance, Constants.DriveTrain.PathPlanner.pathErrorSpikeTolerance);
    // ^how pathplanner reacts to position error
    HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      Constants.DriveTrain.PathPlanner.XYPID.createPIDConstants(),
      Constants.DriveTrain.PathPlanner.thetaPID.createPIDConstants(),
      Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec,
      Math.sqrt(Math.pow((Constants.DriveTrain.Global.distanceBetweenWheels / 2), 2) * 2),
      replanConfig);

    pathConstraints = new PathConstraints(
      Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec, Constants.DriveTrain.Global.maxAcceleration,
      Constants.DriveTrain.Global.maxRotationSpeed, Constants.DriveTrain.Global.maxAccelerationRotation);
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

    inputs = new DriveBaseInputsAutoLogged();
    inputs.currentStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
    inputs.targetStates = new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    inputs.isControlled = false;

    Notifier odometryAndModulesThread = getNotifier();
    odometryAndModulesThread.startPeriodic(1 / Constants.DriveTrain.SwerveModule.modulesThreadHz);
  }

  private @NotNull Notifier getNotifier() {
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

    Notifier odometryAndModulesThread = new Notifier(odometryAndModulesRunnable);
    return odometryAndModulesThread;
  }


  /**
   * resets the robot to 0, 0 and a rotation of 0 (towards red alliance)
   */
  public void resetOnlyDirection(){
    if(DriverStation.getAlliance().isPresent()) if(DriverStation.getAlliance().get() == Alliance.Blue) inputs.currentPose = new Pose2d(inputs.currentPose.getX(), inputs.currentPose.getY(), new Rotation2d());
    else inputs.currentPose = new Pose2d(inputs.currentPose.getX(), inputs.currentPose.getY(), new Rotation2d(-Math.PI));
    else inputs.currentPose = new Pose2d(inputs.currentPose.getX(), inputs.currentPose.getY(), new Rotation2d());

    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

    try {
      odometryLock.lock();
      poseEstimator.resetPosition(new Rotation2d(), positions, inputs.currentPose);
    } finally {
      odometryLock.unlock();
    }

    gyro.reset(inputs.currentPose);
    inputs.targetRotation = inputs.currentPose.getRotation();
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
    SwerveModulePosition positions[] = new SwerveModulePosition[4];
    for(int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

    try {
      odometryLock.lock();
      poseEstimator.resetPosition(new Rotation2d(), positions, newPose);
    } finally {
      odometryLock.unlock();
    }

    gyro.reset(newPose);
    inputs.currentPose = poseEstimator.getEstimatedPosition();
    inputs.targetRotation = inputs.currentPose.getRotation();

    inputs.currentPose = newPose;

    inputs.targetRotation = newPose.getRotation();

    Logger.processInputs(getName(), inputs);
  }

  public void setIsControlled(boolean isControlled){
    inputs.isControlled = isControlled;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * @return if a calculation is controlling the robot's angle and not the last angle of the robot while under user control
   */
  public boolean isControlled(){
    return inputs.isControlled;
  }

  /**
   * gets the acclartion of the robot in the X direction
   * @return the acceleration of the robot in the X direction G
   */
  public double getXAcceleration(){
    return gyro.getAccelerationX();
  }

  /**
   * gets the acclartion of the robot in the Y direction
   * @return the acceleration of the robot in the Y direction G
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
   * gets the target rotation from and angle (from the robot to the target)
   * @param angle the angle to the target
   * @return the target rotation
   */
  public Rotation2d getWantedAngleInCurrentRobotAngle(Rotation2d angle){
   if(getAngle() > 0) return Rotation2d.fromDegrees(getAngle() - (getAngle()%360 + angle.getDegrees()));
   else return Rotation2d.fromDegrees((getAngle() - (getAngle()%360 + angle.getDegrees()))-360);
  }
  
  /**
   * sets the target rotation of the robot's angle (of the theta pid controller)
   * @param alpha the target rotation
   * @param isBeyond360 if the target rotation is beyond 360 degrees
   */
  public void setTargetRotation(Rotation2d alpha, boolean isBeyond360){
    if(isBeyond360) inputs.targetRotation = alpha;
    else inputs.targetRotation = getWantedAngleInCurrentRobotAngle(alpha);
  }

  /**
   * gets the target rotation of the robot's angle
   * @return //the target rotation
   */
  public Rotation2d getTargetRotation(){
    return inputs.targetRotation;
  }

  /**
   * gets the current pose of the robot
   * @return the current pose of the robot
   */
  public Pose2d getPose(){
    return inputs.currentPose;
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
   * calculates the theta value to give to the chassis speed using the target rotation
   * @return the value to give to the drive
   */
  private double calculateTheta(){
    double value = thetaCorrectionController.calculate(getRotation2d().getRadians(), inputs.targetRotation.getRadians());
    if(Math.abs(value) < Constants.DriveTrain.Global.chassisSpeedsDeadZone) return 0;
    return value;
  }

  /**
   * drives the robot with angle PID fix
   * @param Xspeed the speed in the X direction (positive is away from the driver station)
   * @param Yspeed the speed in the Y direction (positive is left)
   * @param rotation the change in the angle of the robot (Positive is to rotate left)
   * @param centerOfRotation the center that the robot rotates about
   */
  public void drive(double Xspeed, double Yspeed, double rotation, Translation2d centerOfRotation){
    inputs.rotationSpeedInputBeforePID = rotation; //logs the rotation speed before PID

    if(rotation == 0) rotation = calculateTheta();
    else if(!isControlled()){
      inputs.targetRotation = getRotation2d();
    }

    inputs.targetStates = driveTrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotation, getRotation2d()), centerOfRotation); //calculates the target states
    SwerveDriveKinematics.desaturateWheelSpeeds(inputs.targetStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec); //desaturates the wheel speeds (to make sure none of the wheel exceed the max speed)

    for(int i = 0; i < 4; i++){
      inputs.targetStates[i] = modules[i].run(inputs.targetStates[i]); //sets the module state
    }

    inputs.XspeedInput = Xspeed; //logs the X speed before PID
    inputs.YspeedInput = Yspeed; //logs the Y speed before PID
    inputs.rotationSpeedInputAfterPID = rotation;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot relative to itself
   * @param Xspeed the X speed of the robot (forward is positive) m/s
   * @param Yspeed the Y speed of the robot (left is positive) m/s
   * @param rotation  the rotation of the robot (left is positive) rad/s
   */
  public void robotRelativeDrive(double Xspeed, double Yspeed, double rotation){
    if(rotation == 0) rotation = calculateTheta();
    else if(!isControlled()){
      inputs.targetRotation = getRotation2d();
    }

    inputs.targetStates = driveTrainKinematics.toSwerveModuleStates(new ChassisSpeeds(Xspeed, Yspeed, rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(inputs.currentStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec);

    for(int i = 0; i < 4; i++){
      inputs.targetStates[i] = modules[i].run(inputs.targetStates[i]);
    }

    inputs.XspeedInput = Xspeed;
    inputs.YspeedInput = Yspeed;
    inputs.rotationSpeedInputBeforePID = rotation;
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
    inputs.targetStates = driveTrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(inputs.targetStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec);

    for(int i = 0; i < 4; i++){
      inputs.targetStates[i] = modules[i].run(inputs.targetStates[i]);
    }

    inputs.XspeedInput = chassisSpeeds.vxMetersPerSecond;
    inputs.YspeedInput = chassisSpeeds.vyMetersPerSecond;
    inputs.rotationSpeedInputBeforePID = chassisSpeeds.omegaRadiansPerSecond;
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
    inputs.targetStates = driveTrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotation, getRotation2d()), centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(inputs.targetStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec);

    for(int i = 0; i < 4; i++){
      inputs.targetStates[i] = modules[i].run(inputs.targetStates[i]);
    }

    inputs.XspeedInput = Xspeed;
    inputs.YspeedInput = Yspeed;
    inputs.rotationSpeedInputBeforePID = rotation;
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
    return AutoBuilder.pathfindToPose(targetPose, pathConstraints, endVelocity, rotationDelay);
  }

  /**
   * path finds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
   * @return a command that follows a path to the target pose
   */
  public Command findPath(Pose2d targetPose, double endVelocity){
    return AutoBuilder.pathfindToPose(targetPose, pathConstraints, endVelocity);
  }

  /**
   * path finds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @return a command that follows a path to the target pose
   */
  public Command findPath(Pose2d targetPose){
    return AutoBuilder.pathfindToPose(targetPose, pathConstraints);
  }

  /**
   * path finds to a given path then follows that path
   * @param targetPath the path to path find to and follow
   * @return a command that path finds to a given path then follows that path
   */
  public Command pathFindToPathAndFollow(PathPlannerPath targetPath){
    return AutoBuilder.pathfindThenFollowPath(targetPath, pathConstraints);
  }

  /**
   * path finds to a given path then follows that path
   * @param targetPath the path to path find to and follow
   * @param rotationDelay the delay in meters in which the robot does not change it's holonomic angle
   * @return a command that path finds to a given path then follows that path
   */
  public Command pathFindToPathAndFollow(PathPlannerPath targetPath, double rotationDelay){
    return AutoBuilder.pathfindThenFollowPath(targetPath, pathConstraints, rotationDelay);
  }

  /**
   * this updates the states of the modules, call this function periodically
   */
  public void update(){
      for(int i = 0; i < 4; i++){
        inputs.currentStates[i] = modules[i].getSwerveModuleState();
      }

    try {
      odometryLock.lock();
      inputs.currentPose = poseEstimator.getEstimatedPosition();
    } finally {
      odometryLock.unlock();
    }
    
    RobotContainer.field.setRobotPose(inputs.currentPose);

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
      driveBase.setIsControlled(false);
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
        (Math.abs(controller.getLeftY()) > 0.05 ? -controller.getLeftY() : 0) * Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec,

        (Math.abs(controller.getLeftX()) > 0.05 ? controller.getLeftX() : 0) * -Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec,

        (Math.abs(controller.getRightX()) > 0.05 ? controller.getRightX() : 0) * -Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec
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


  public static class lockSwerveInXPatternCommand extends Command{
    DriveBase driveBase;
    public lockSwerveInXPatternCommand(){
      driveBase = RobotContainer.driveBase;
      addRequirements(driveBase);
    }

    @Override
    public void initialize(){
//      for(int i = 0; i < 4; i++) driveBase.modules[i].goToXPosition();
    }

    @Override
    public void end(boolean interrupted) { driveBase.drive(0, 0, 0);}
  }

  private static class SysID{
    public static enum SysIDType{
      Steer,
      Drive,
      XY,
      Theta
    }
    SysIdRoutine steerSysId;
    SysIdRoutine driveSysId;
    SysIdRoutine xySpeedSysId;
    SysIdRoutine thetaSpeedSysId;

    private SysID(DriveBase driveBase){
      steerSysId = new SysIdRoutine(
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                      (voltage) -> {
                        for(int i = 0; i < 4; i++) driveBase.modules[i].runSysID(null, voltage);
                      },
                      null,
                      driveBase,
                      "steerSysId"
              ));

      driveSysId = new SysIdRoutine(
              new SysIdRoutine.Config(),
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

      xySpeedSysId = new SysIdRoutine(
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                      (voltage) -> {
                        driveBase.drive(voltage.in(Units.Volts) / 3, 0, 0);
                      },
                      (log) -> {
                        ChassisSpeeds chassisSpeeds = driveBase.getChassisSpeeds();
                        Pose2d pose = driveBase.getPose();
                        log.motor("robotX").linearAcceleration(Units.MetersPerSecondPerSecond.of(driveBase.getXAcceleration() * 9.80665)).linearVelocity(Units.MetersPerSecond.of(chassisSpeeds.vxMetersPerSecond)).linearPosition(Units.Meters.of(pose.getX()));
                        log.motor("robotY").linearAcceleration(Units.MetersPerSecondPerSecond.of(driveBase.getYAcceleration() * 9.80665)).linearVelocity(Units.MetersPerSecond.of(chassisSpeeds.vyMetersPerSecond)).linearPosition(Units.Meters.of(pose.getY()));
                      },
                      driveBase,
                      "xySpeedSysId"
              ));

      double[] prevAngleVelocity = {0};
      double[] preAngleVelocityTimeStamp = {0};

      thetaSpeedSysId = new SysIdRoutine(
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                      (voltage) -> {
                        driveBase.drive(0, 0, voltage.in(Units.Volts));
                      },
                      (log) -> {
                        ChassisSpeeds chassisSpeeds = driveBase.getChassisSpeeds();
                        Pose2d pose = driveBase.getPose();
                        double angleAccl = (chassisSpeeds.omegaRadiansPerSecond - prevAngleVelocity[0]) / (Timer.getFPGATimestamp() - preAngleVelocityTimeStamp[0]);
                        prevAngleVelocity[0] = chassisSpeeds.omegaRadiansPerSecond;
                        preAngleVelocityTimeStamp[0] = Timer.getFPGATimestamp();
                        log.motor("robotTheta").angularPosition(Units.Radians.of(pose.getRotation().getRadians())).angularVelocity(Units.RadiansPerSecond.of(chassisSpeeds.omegaRadiansPerSecond)).angularAcceleration(Units.RadiansPerSecond.per(Units.Seconds).of(angleAccl));
                      },
                      driveBase,
                      "thetaSpeedSysId"
              ));
    }

    public Command getSysIDCommand(SysIDType type, boolean isDynamic, boolean isForward){
      return isDynamic ? switch (type) {
        case Steer -> steerSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Drive -> driveSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case XY -> xySpeedSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Theta -> thetaSpeedSysId.dynamic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        default -> null;
      }
      : switch (type) {
        case Steer -> steerSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Drive -> driveSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case XY -> xySpeedSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        case Theta -> thetaSpeedSysId.quasistatic(isForward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
        default -> null;
      };
    }
  }
}
