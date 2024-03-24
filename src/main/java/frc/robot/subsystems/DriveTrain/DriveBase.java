// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * The DriveBase class represents the drivetrain of the robot.
 * It controls the movement and positioning of the robot using swerve drive.
 */
public class DriveBase extends SubsystemBase {
  SwerveModule[] modules = new SwerveModule[4]; //the array of the modules
  
  SwerveModuleState[] currentStates = new SwerveModuleState[4]; //the current states of the modules
  SwerveModuleState[] targetStates = new SwerveModuleState[4]; //the target states of the modules

  SwerveModulePosition[] currentPostions = new SwerveModulePosition[4]; //the postions of the modules

  SwerveDriveKinematics driveTrainKinematics; //the kinematics of the swerve drivetrain

  AHRS Navx; //the gyro device
  
  double navxOffset;
  SwerveDrivePoseEstimator poseEstimator; //the pose estimator of the drivetrain

  PIDController thetaCorrectionController; //the pid controller that fixes the angle of the robot
  Pose2d currentPose; //the current pose2d of the robot
  Rotation2d targetRotation; //the target rotation of the robot

  HolonomicPathFollowerConfig pathFollowerConfig; //the config for path following a path
  ReplanningConfig replanConfig; //the config for when to replan a path
  PathConstraints pathConstraints; //the constraints for pathPlanner

  List<Pair<Translation2d, Translation2d>> deadZoneList = new ArrayList<Pair<Translation2d, Translation2d>>();
  //A list containg boxes of dead zone for path Planner

  DriveBaseInputsAutoLogged inputs; //an object represnting the logger class

  Field2d field; //the field object for the smart dashboard

  Command swerveXPattern; //makes the swerve in an X pattern (overrides default command)

  @AutoLog
  public static class DriveBaseInputs{
    double XspeedInput = 0; //the X speed input
    double YspeedInput = 0; //the Y speed input

    double rotationSpeedInputBeforePID = 0; //the rotation speed input before PID
    double rotationSpeedInputAfterPID = 0; //the rotation speed input after PID

    Pose2d currentPose = new Pose2d(); //the current pose of the robot
    Rotation2d targetRotation = new Rotation2d(); //the target rotation of the robot (in radians)

    Rotation2d chassisAngle = new Rotation2d(); //the angle of the robot

    SwerveModuleState[] currentStates = new SwerveModuleState[4]; //the current states of the modules
    SwerveModuleState[] targetStates = new SwerveModuleState[4]; //the target states of the modules

    boolean isControlled;
  }


  /** Creates a new DriveBase. */
  public DriveBase() {
    modules[0] = new SwerveModule(Constants.DriveTrain.front_left); //the front left module
    modules[1] = new SwerveModule(Constants.DriveTrain.front_right); //the front right module
    modules[2] = new SwerveModule(Constants.DriveTrain.back_left); //the back left module
    modules[3] = new SwerveModule(Constants.DriveTrain.back_right); //the back right module

    for(int i = 0; i < 4; i++){
      currentStates[i] = new SwerveModuleState(); //creates the zerod states
      targetStates = currentStates; //copies it
      currentPostions[i] = new SwerveModulePosition(); //creates the zerod postions
    }

    Navx = new AHRS();
    Navx.reset(); //resets the gyro at the start 

    SmartDashboard.putData("Navx", Navx);

    driveTrainKinematics = new SwerveDriveKinematics(Constants.DriveTrain.SwerveModule.moduleTranslations);

    // poseEstimator = new SwerveDrivePoseEstimator(driveTrainKinematics, new Rotation2d(), currentPostions, new Pose2d(), Constants.DriveTrain.Vision.stateStdDevs, Constants.DriveTrain.Vision.visionStdDevs); //creates a new pose estimator class with given values
    poseEstimator = new SwerveDrivePoseEstimator(driveTrainKinematics, new Rotation2d(), currentPostions, new Pose2d()); //creates a new pose estimator class with given value
    navxOffset = 0;

    thetaCorrectionController = Constants.DriveTrain.Global.thetaCorrectionPID.createPIDController(); //creates the pid controller of the robots angle

    targetRotation = new Rotation2d(); //creates a new target rotation

    replanConfig = new ReplanningConfig(Constants.DriveTrain.PathPlanner.planPathTostartingPointIfNotAtIt, Constants.DriveTrain.PathPlanner.enableDynamicReplanning, Constants.DriveTrain.PathPlanner.pathErrorTolerance, Constants.DriveTrain.PathPlanner.pathErrorSpikeTolerance);
    // ^how pathplanner reacts to postion error
    pathFollowerConfig = new HolonomicPathFollowerConfig(
      Constants.DriveTrain.PathPlanner.XYPID.createPIDConstants(),
      Constants.DriveTrain.PathPlanner.thetaPID.createPIDConstants(),
      Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec,
      Math.sqrt(Math.pow((Constants.DriveTrain.Global.distanceBetweenWheels / 2), 2) * 2),
      replanConfig);

    pathConstraints = new PathConstraints(
      Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec, Constants.DriveTrain.Global.maxAcceleration,
      Constants.DriveTrain.Global.maxRotationSpeed, Constants.DriveTrain.Global.maxAccelerationRotation);
    //^creates path constraints for pathPlanner

    //creates a trigger that will configure the autobuilder if the allince is set and the autobuilder is not configured
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::reset,
      this::getChassisSpeeds,
      this::drive,
      pathFollowerConfig,
      () -> {if(DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
      else return false;},
      this);
    //^configures the autobuilder for pathPlanner

    // new Trigger(RobotState::isAutonomous).onTrue(new InstantCommand(() -> resetOnlyDirection())); //triggers a postion and state reset when the robot starts the auto period

    new Trigger(RobotState::isEnabled).whileTrue(new StartEndCommand(() -> //relaeses or holds the motors if the robot is enables or not
      setMoudlesBrakeMode(true)
    , () -> 
      {if(!DriverStation.isFMSAttached()) setMoudlesBrakeMode(false);}
    ).ignoringDisable(true));

    new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() -> 
    this.setDefaultCommand(new DriveCommand()),
    () -> {
      this.removeDefaultCommand();
    }).ignoringDisable(true));

    inputs = new DriveBaseInputsAutoLogged();
    for(int i = 0; i < 4; i++){
      inputs.currentStates[i] = currentStates[i];
      inputs.targetStates[i] = targetStates[i];
    }

    field = new Field2d();
    SmartDashboard.putData(field);
    inputs.isControlled = false;
  }


  /**
   * resets the robot to a new pose
   * @param newPose the new pose the robot should be in
   */
  public void resetOnlyDirection(){
    if(DriverStation.getAlliance().isPresent()) if(DriverStation.getAlliance().get() == Alliance.Blue) currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());
    else currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(-Math.PI));
    else currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getNavxAngle()), currentPostions, currentPose);
    targetRotation = currentPose.getRotation();
    navxOffset = -getNavxAngle();
  }

  public void setMoudlesBrakeMode(boolean isBrake){
    for(int i = 0; i < 4; i++){
      modules[i].setBrakeMode(isBrake);
    }
  }


  /**
   * resets the robot to a new pose
   */
  public void Reset(){
    Pose2d startingPose2d;

    try {
      switch(DriverStation.getAlliance().get()){ //chooses the starting postions depends on which alliance is the driver station in
        default:
          startingPose2d = new Pose2d();
          break;

        case Blue:
          startingPose2d = new Pose2d();
          break;

        case Red:
          startingPose2d = new Pose2d(16.5, 8, Rotation2d.fromRadians(Math.PI));
          break;
      }
    }
    catch (NoSuchElementException e) {
      startingPose2d = new Pose2d();
    }

    poseEstimator.resetPosition(Rotation2d.fromDegrees(getNavxAngle()), currentPostions, startingPose2d); //reset poseEstimator with the new starting postion
    navxOffset = -(startingPose2d.getRotation().getDegrees() - getNavxAngle());
    currentPose = poseEstimator.getEstimatedPosition();
    targetRotation = currentPose.getRotation();

    inputs.currentPose = startingPose2d;

    inputs.targetRotation = startingPose2d.getRotation();
    
    inputs.chassisAngle= startingPose2d.getRotation();

    Logger.processInputs(getName(), inputs);
  }

  /**
   * resets the robot to a new pose
   * @param newPose the new pose the robot should be in
   */
  public void reset(Pose2d newPose){
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getNavxAngle()), currentPostions, newPose); //reset poseEstimator with the new starting postion
    navxOffset = -(newPose.getRotation().getDegrees() - getNavxAngle());
    currentPose = poseEstimator.getEstimatedPosition();
    targetRotation = currentPose.getRotation();

    inputs.currentPose = newPose;

    inputs.targetRotation = newPose.getRotation();

    inputs.chassisAngle = getRotation2d();

    Logger.processInputs(getName(), inputs);
  }

  public void setIsControlled(boolean isControlled){
    inputs.isControlled = isControlled;
    Logger.processInputs(getName(), inputs);
  }

  public boolean isControlled(){
    return inputs.isControlled;
  }

  /**
   * returns the reported Rotation by the odometry
   * @return Rotation2d report the the odometry
   */
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getAngle());
  }

  /**
   * returns the current angle of the robot
   * @return the angle of the robot (left is positive) IN DEGREES
   */
  public double getAngle(){
    return getNavxAngle() + navxOffset;
  }

  /**
   * gets the angle of the navx direeclty (only use for pose estimore, for the rest use GetAngle())
   * @return the angle of the navx
   */
  public double getNavxAngle(){
    return Navx.getAngle();
  }

  /**
   * gets the current chassisSpeeds of the robot
   * @return the current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds(){
    return driveTrainKinematics.toChassisSpeeds(currentStates);
  }

  /**
   * gets the current absulete (field relative ) speeds of the robot
   * @return the current chassis speeds
   */
  public ChassisSpeeds getAbsoluteChassisSpeeds(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
  }

  /**
   * gets the field object of the drivebase
   * @return the field object
   */
  public Field2d getField2d(){
    return field;
  }

  /**
   * gets the target rotation from and angle (from the robot to the target)
   * @param angle the angle in degrees
   * @return the target rotation (in dgrees)
   */
  public double getTargetAngleInRobotAngle(double angle){
    return getAngle() - (getAngle()%360 + angle);
  }

  /**
   * gets the target rotation from and angle (from the robot to the target)
   * @param angle the angle
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
    if(isBeyond360) targetRotation = alpha;
    else targetRotation = getWantedAngleInCurrentRobotAngle(alpha);

    inputs.targetRotation = targetRotation;
  }

  /**
   * gets the target rotation of the robot's angle
   * @return //the target rotation
   */
  public Rotation2d getTargetRotation(){
    return targetRotation;
  }

  /**
   * gets the current pose of the robot
   * @return the current pose of the robot
   */
  public Pose2d getPose(){
    return currentPose;
  }

  /**
   * updates odemtry with vision mesurments
   * @param visionPose the pose of the robot from vision
   * @param timeStamp the time stamp of the vision mesurment
   */
  public void addVisionMesrument(Pose2d visionPose, double timeStamp){
    poseEstimator.addVisionMeasurement(visionPose, timeStamp);
  }

  /**
   * calculates the value to give to the chassis speed
   * @param setPoint a new target rotation
   * @return returns the value to give to the chassis speed constructer
   */
  public double calculateTheta(Rotation2d setPoint){
    targetRotation = setPoint;
    double value = thetaCorrectionController.calculate(getRotation2d().getRadians(), setPoint.getRadians());
    if(Math.abs(value) < Constants.DriveTrain.Global.chassisSpeedsDeadZone) return 0;
    return value;
  }

  /**
   * calculates the theta value to give to the chassis speed using the target rotation
   * @return the value to give to the drive
   */
  public double calculateTheta(){
    double value = thetaCorrectionController.calculate(getRotation2d().getRadians(), targetRotation.getRadians());
    if(Math.abs(value) < Constants.DriveTrain.Global.chassisSpeedsDeadZone) return 0;
    return value;
  }

  /**
   * drives the robot with angle PID fix
   * @param Xspeed the speed in the X diraction (postive is away from the driver station)
   * @param yspeed the speed in the Y diraction (postive is left)
   * @param rotation the change in the angle of the robot (Postive is to rotate left)
   * @param centerOfRotation the center that the robot rotates about
   */
  public void drive(double Xspeed, double Yspeed, double rotation, Translation2d centerOfRotation){
    inputs.rotationSpeedInputBeforePID = rotation; //logs the rotation speed before PID
    // if(rotation == 0) rotation = calculateTheta();
    // else{
    //   targetRotation = getRotation2d();
    //   inputs.targetRotation = targetRotation;
    // }
    if(rotation == 0) rotation = calculateTheta();
    else if(!isControlled()){
      targetRotation = getRotation2d();
      inputs.targetRotation = targetRotation;
    }

    targetStates = driveTrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotation, getRotation2d()), centerOfRotation); //calulates the target states
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec); //desaturates the wheel speeds (to make sure none of the wheel exceed the max speed)

    for(int i = 0; i < 4; i++){
      targetStates[i] = SwerveModuleState.optimize(targetStates[i], modules[i].getCurrentState().angle); //optimizes the target states (to make sure the wheels don't rotate more than 90 degrees)
      modules[i].setModuleState(targetStates[i]); //sets the module state
    }

    inputs.targetStates = targetStates; //updates the logger target states
    inputs.XspeedInput = Xspeed; //logs the X speed before PID
    inputs.YspeedInput = Yspeed; //logs the Y speed before PID
    inputs.rotationSpeedInputAfterPID = rotation;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot relative to it self
   * @param Xspeed the xseed of the robot (forward is positive) m/s
   * @param Yspeed the yseed of the robot (left is positive) m/s
   * @param rotation  the rotation of the robot (left is positive) rad/s
   */
  public void robotRelativeDrive(double Xspeed, double Yspeed, double rotation){
    if(rotation == 0) rotation = calculateTheta();
    else if(!isControlled()){
      targetRotation = getRotation2d();
      inputs.targetRotation = targetRotation;
    }

    targetStates = driveTrainKinematics.toSwerveModuleStates(new ChassisSpeeds(Xspeed, Yspeed, rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(currentStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec);

    for(int i = 0; i < 4; i++){
      targetStates[i] = SwerveModuleState.optimize(targetStates[i], currentStates[i].angle);
      modules[i].setModuleState(targetStates[i]);
    }

    inputs.targetStates = targetStates;
    inputs.XspeedInput = Xspeed;
    inputs.YspeedInput = Yspeed;
    inputs.rotationSpeedInputBeforePID = rotation;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot with angle PID fix
   * @param Xspeed the speed in the X diraction (postive is away from the driver station)
   * @param yspeed the speed in the Y diraction (postive is left)
   * @param rotation the change in the angle of the robot (Postive is to rotate left)
   */
  public void drive(double Xspeed, double Yspeed, double rotation){drive(Xspeed, Yspeed, rotation, new Translation2d());}

  /**
   * drives the robot without built in pid fixes
   * @param chassisSpeeds the chassis speeds of the target
   */
  public void drive(ChassisSpeeds chassisSpeeds){
    targetStates = driveTrainKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec);

    for(int i = 0; i < 4; i++){
      targetStates[i] = SwerveModuleState.optimize(targetStates[i], currentStates[i].angle);
      modules[i].setModuleState(targetStates[i]);
    }

    inputs.targetStates = targetStates;
    inputs.XspeedInput = chassisSpeeds.vxMetersPerSecond;
    inputs.YspeedInput = chassisSpeeds.vyMetersPerSecond;
    inputs.rotationSpeedInputBeforePID = chassisSpeeds.omegaRadiansPerSecond;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot without any PID control at all
   * @param Xspeed the speed in the X diraction (postive is away from the driver station)
   * @param Yspeed the speed in the Y diraction (postive is left)
   * @param rotation the change in the angle of the robot (Postive is to rotate left)
   * @param centerOfRotation the center that the robot rotates about
   */
  public void driveWithOutPID(double Xspeed, double Yspeed, double rotation, Translation2d centerOfRotation){
    targetStates = driveTrainKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotation, getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec);

    for(int i = 0; i < 4; i++){
      targetStates[i] = SwerveModuleState.optimize(targetStates[i], currentStates[i].angle);
      modules[i].setModuleState(targetStates[i]);
    }

    inputs.targetStates = targetStates;
    inputs.XspeedInput = Xspeed;
    inputs.YspeedInput = Yspeed;
    inputs.rotationSpeedInputBeforePID = rotation;
    Logger.processInputs(getName(), inputs);
  }

  /**
   * drives the robot without any PID control at all
   * @param Xspeed the speed in the X diraction (postive is away from the driver station)
   * @param Yspeed the speed in the Y diraction (postive is left)
   * @param rotation the change in the angle of the robot (Postive is to rotate left)
   */
  public void driveWithOutPID(double Xspeed, double Yspeed, double rotation){ driveWithOutPID(Xspeed, Yspeed, rotation, new Translation2d());}


  /**
   * pathfinds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
   * @param rotationDelay the delay in meters before the robot starts rotationg
   * @return a command that followes a path to the target pose
   */
  public Command findPath(Pose2d targetPose, double endVelocity ,double rotationDelay){
    return AutoBuilder.pathfindToPose(targetPose, pathConstraints, endVelocity, rotationDelay);
  }

  /**
   * pathfinds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
   * @return a command that followes a path to the target pose
   */
  public Command findPath(Pose2d targetPose, double endVelocity){
    return AutoBuilder.pathfindToPose(targetPose, pathConstraints, endVelocity);
  }

  /**
   * pathfinds a path from the current pose to the target pose
   * @param targetPose the target pose
   * @return a command that followes a path to the target pose
   */
  public Command findPath(Pose2d targetPose){
    return AutoBuilder.pathfindToPose(targetPose, pathConstraints);
  }

  /**
   * pathfinds to a given path then follows that path
   * @param targetPath the path to pathfind to and follow
   * @return a command that pathfinds to a given path then follows that path
   */
  public Command pathFindToPathAndFollow(PathPlannerPath targetPath){
    return AutoBuilder.pathfindThenFollowPath(targetPath, pathConstraints);
  }

  /**
   * pathfinds to a given path then follows that path
   * @param targetPath the path to pathfind to and follow
   * @param rotationDelay the delay in meters in which the robot does not change it's holonomic angle
   * @return a command that pathfinds to a given path then follows that path
   */
  public Command pathFindToPathAndFollow(PathPlannerPath targetPath, double rotationDelay){
    return AutoBuilder.pathfindThenFollowPath(targetPath, pathConstraints, rotationDelay);
  }

  /**
   * creates a command to follow a given path
   * will not pathfind to the path
   * will reset the currents robot pose to the start of the path
   * USE ONLY AS THE FIRST PATH IF NEEDED
   * I WILL BEAT YOU UP WITH A STICK IF YOU ASK ME WHY IT THINKS IT'S IN THE WRONG PLACE
   * @param path the path to follow
   * @return a command that follows that path
   */
  public Command followPath(PathPlannerPath path){
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getNavxAngle()), currentPostions, path.getPreviewStartingHolonomicPose());
    currentPose = poseEstimator.getEstimatedPosition();
    targetRotation = path.getGoalEndState().getRotation();
    return AutoBuilder.followPath(path);
  }

  /**
   * this updates the states of the modules, call this funcation peridaclly
   */
  public void update(){
    for(int i = 0; i < 4; i++){
      modules[i].update();
      currentStates[i] = modules[i].getCurrentState();
      currentPostions[i] = modules[i].getModulePosition();
    }
    poseEstimator.update(Rotation2d.fromDegrees(getNavxAngle()), currentPostions);    
    currentPose = poseEstimator.getEstimatedPosition();
    // currentPose = new Pose2d(currentPose.getTranslation(), getRotation2d());
    
    field.setRobotPose(currentPose);

    // SmartDashboard.putData(field);
    // SmartDashboard.putData(Navx);

    inputs.currentPose = currentPose;

    inputs.currentStates = currentStates;

    inputs.chassisAngle = getRotation2d();

    Logger.processInputs(getName(), inputs);
  }

  @Override
  public void periodic() {
    update();
  }


  public static class DriveCommand extends Command{
    DriveBase driveBase;
    CommandPS5Controller controller;

    public DriveCommand(){
      driveBase = RobotContainer.driveBase;
      controller = RobotContainer.driveController;

      addRequirements(driveBase);
    }

    @Override
    public void initialize() {
      RobotContainer.driveBase.drive(0, 0, 0);
      RobotContainer.driveBase.setIsControlled(false);;
    }

    @Override
    public void execute() {

      RobotContainer.driveBase.drive(
        //this basiclly takes the inputs from the controller and firsts checks if it's not drift or a mistake by checking if it is above a certain value then it multiplies it by the R2 axis that the driver uses to control the speed of the robot
        (Math.abs(controller.getLeftY()) > 0.05 ? -controller.getLeftY() : 0) * (controller.getR2Axis() > 0.1 ? 1 + (Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec - 1) * (1 - controller.getR2Axis()) : 1),

        (Math.abs(controller.getLeftX()) > 0.05 ? controller.getLeftX() : 0) * (controller.getR2Axis() > 0.1 ? 1 + (Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec - 1) * (1 - controller.getR2Axis()) : 1),

        Math.abs(controller.getRightX()) > 0.05 ? controller.getRightX() : 0
        );
    }

    @Override
    public void end(boolean interrupted) {
      RobotContainer.driveBase.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
      return RobotState.isDisabled();
    }
  }


  public static class lockSwerveInXPatternCommand extends Command{
    private static lockSwerveInXPatternCommand instance;
    private lockSwerveInXPatternCommand(){
      addRequirements(RobotContainer.driveBase);
    }

    @Override
    public void initialize(){ 
      for(int i = 0; i < 4; i++) RobotContainer.driveBase.modules[i].goToXPostion();
    }

    public static Command getInstance(){
      if(instance == null) instance = new lockSwerveInXPatternCommand();
      return instance;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
