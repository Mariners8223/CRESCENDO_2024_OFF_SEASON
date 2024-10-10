package frc.robot.subsystems.Vision;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.subsystems.Vision.Camera.ArduoCamIO;
import frc.robot.subsystems.Vision.Camera.CameraIO;
import frc.robot.subsystems.Vision.Camera.CameraInputsAutoLogged;
import frc.robot.subsystems.Vision.Camera.LogitechIO;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.Vision.VisionConstants.CameraLocation;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision.VisionConstants.SpeakerConstants;

public class Vision extends SubsystemBase {

    private final VisionConstants.CameraLocation[] cameraLocations = VisionConstants.CameraLocation.values();

    private final CameraIO[] cameras = new CameraIO[cameraLocations.length];
    private final CameraInputsAutoLogged[] inputs = new CameraInputsAutoLogged[cameraLocations.length];


    private final Consumer<Pair<Pose2d, Double>> poseConsumer;

    private SpeakerConstants speakerConstants = SpeakerConstants.BLUE;
    private final double ROBOT_FRAME_HEIGHT;

    private final Supplier<Pose2d> currentPose;

    public Vision(Supplier<Pose2d> referencePose, Consumer<Pair<Pose2d, Double>> poseConsumer, double robotFrameHeight) {

        for (int i = 0; i < cameraLocations.length; i++) {
            cameras[i] = cameraLocations[i].HAS_POSE_ESTIMATION ?
                    new ArduoCamIO(cameraLocations[i], referencePose) :
                    new LogitechIO(cameraLocations[i].NAME);
        }

        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new CameraInputsAutoLogged();
        }

        this.poseConsumer = poseConsumer;
        this.currentPose = referencePose;

        ROBOT_FRAME_HEIGHT = robotFrameHeight;

        new Trigger(RobotState::isAutonomous).onTrue(new InstantCommand(() -> {

            if (DriverStation.getAlliance().isPresent() &&
                    DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                speakerConstants = SpeakerConstants.BLUE;

            else speakerConstants = SpeakerConstants.RED;

        }).ignoringDisable(true));
    }

    public void setPipeline(int pipelineID, CameraLocation cameraLocation) {
        cameras[cameraLocation.ordinal()].setPipeline(pipelineID);
    }

    public VisionOutPuts getAngleToSpeakerFront(double offsetX, double offsetZ, double speedMultiplier) {
        VisionOutPuts result = getAngleToSpeaker(CameraLocation.FRONT_RIGHT, offsetX, offsetZ, true, speedMultiplier);

        Logger.recordOutput("Vision/" + CameraLocation.FRONT_RIGHT + "/yaw to speaker", result.getYaw().getRadians());
        Logger.recordOutput("Vision/" + CameraLocation.FRONT_RIGHT + "/pitch to speaker", result.getPitch().in(Units.Radians));
        Logger.recordOutput("Vision/" + CameraLocation.FRONT_RIGHT + "/velocity needed", result.getSpeed().in(Units.RPM));

        return result;
    }

    public VisionOutPuts getAngleToSpeakerBack(double offsetX, double offsetZ, double speedMultiplier) {
        VisionOutPuts result = getAngleToSpeaker(CameraLocation.BACK_LEFT, offsetX, offsetZ, false, speedMultiplier);

        Logger.recordOutput("Vision/" + CameraLocation.BACK_LEFT + "/yaw to speaker", result.getYaw().getRadians());
        Logger.recordOutput("Vision/" + CameraLocation.BACK_LEFT + "/pitch to speaker", result.getPitch().in(Units.Radians));
        Logger.recordOutput("Vision/" + CameraLocation.BACK_LEFT + "/velocity needed", result.getSpeed().in(Units.RPM));

        return result;
    }

    private VisionOutPuts getAngleToSpeaker
            (CameraLocation camera, double offsetX, double offsetZ, boolean facingFront, double speedMultiplier) {

        if (camera == CameraLocation.FRONT) return new VisionOutPuts(0, 0, 0);

        var input = inputs[camera.ordinal()];

        if (!input.hasTarget)
            return calculateAngleToSpeakerBasedOdemetry(currentPose.get(), offsetX, offsetZ, facingFront, speedMultiplier);

        int index = -1;

        for (int i = 0; i < 4; i++) {
            if (input.targetID[i] == speakerConstants.SPEAKER_CENTER_ID) {
                index = i;
                break;
            }
        }

        if (index == -1)
            return calculateAngleToSpeakerBasedOdemetry(currentPose.get(), offsetX, offsetZ, facingFront, speedMultiplier);

        return calculateAngleToSpeakerBasedVision(camera.CAMERA_TO_ROBOT,
                input.xAngleDegrees[index], input.yAngleDegrees[index], offsetX, offsetZ, speedMultiplier);
    }

    /**
     * calculates the angle to the speaker based on the vision data
     *
     * @param cameraToRobot the transformation of the center of the robot to the camera
     * @param angleX        the yaw to the april tag
     * @param angleY        the pitch to the april tag
     * @param offsetX       the offset to add distance from the center of the robot on the x-axis
     * @param offsetZ       the offset to add distance from the center of the robot on the z-axis
     * @return the angle to the speaker (only pitch and yaw)
     */
    private VisionOutPuts calculateAngleToSpeakerBasedVision
    (Transform3d cameraToRobot, double angleX,
     double angleY, double offsetX, double offsetZ, double speedMultiplier) {

        double angleYToAprilTag = degreesToRadians(angleY) - cameraToRobot.getRotation().getY();

        double xDistCamera =
                (speakerConstants.SPEAKER_CENTER_APRIL_TAG.getZ() - ROBOT_FRAME_HEIGHT + cameraToRobot.getZ()) /
                        Math.tan(angleYToAprilTag);

        double xDistToRobotCenter = xDistCamera + Math.abs(cameraToRobot.getX());

        double yDistRobotCenter = Math.tan(-degreesToRadians(angleX) + cameraToRobot.getRotation().getZ())
                * xDistCamera - Math.abs(cameraToRobot.getY());

//        double distanceToTarget
//                = Math.sqrt(xDistToRobotCenter * xDistToRobotCenter + yDistRobotCenter * yDistRobotCenter) + offsetX;

        double distanceToTarget = Math.hypot(xDistToRobotCenter, yDistRobotCenter) + offsetX;

        double pitch =
                Math.atan2(speakerConstants.SPEAKER_TARGET.getZ() - ROBOT_FRAME_HEIGHT - offsetZ,
                        distanceToTarget);

        double yaw = Math.atan2(yDistRobotCenter, xDistToRobotCenter);

        return new VisionOutPuts(pitch, yaw, distanceToTarget * speedMultiplier);
    }

    private double degreesToRadians(double degrees){
        return degrees * (Math.PI / 180);
    }

    /**
     * calculates the angle to the speaker based on the odemetry
     *
     * @param currentPose the current pose of the robot
     * @param offsetX     the offset to add distance from the center of the robot on the x-axis
     * @param offsetZ     the offset to add distance from the center of the robot on the z-axis
     * @return the angle to the speaker (only pitch and yaw)
     */
    private VisionOutPuts calculateAngleToSpeakerBasedOdemetry(
            Pose2d currentPose, double offsetX, double offsetZ, boolean facingFront, double speedMultiplier) {

        double lerp = 1 -
                ((currentPose.getY() - speakerConstants.ROBOT_Y_START) /
                        (speakerConstants.ROBOT_Y_END - speakerConstants.ROBOT_Y_START));


        double targetY = lerp > 1 ? speakerConstants.SPEAKER_Y_END :
                (speakerConstants.SPEAKER_Y_END - speakerConstants.SPEAKER_Y_START)
                        * lerp + speakerConstants.SPEAKER_Y_START;

        // double targetY = speakerConstants.SPEAKER_TARGET.getY();

        double targetX = speakerConstants.SPEAKER_TARGET.getX();

        double xDist = targetX - currentPose.getX();

        double yDist = targetY - currentPose.getY();

        double yaw = -Math.atan(yDist / xDist);

        double robotRelativeYaw =
                facingFront ? Math.PI - yaw - MathUtil.angleModulus(currentPose.getRotation().getRadians())
                        : yaw;

        double distanceToTarget = Math.sqrt(xDist * xDist + yDist * yDist) - offsetX;

        // double pitch = Math.atan2(speakerConstants.SPEAKER_TARGET.getZ() - ROBOT_FRAME_HEIGHT - offsetZ,
        //         distanceToTarget);

        double pitch = Math.atan((speakerConstants.SPEAKER_TARGET.getZ() - ROBOT_FRAME_HEIGHT - offsetZ) / distanceToTarget);

        Logger.recordOutput("Speaker Target", new Translation2d(targetX, targetY));

        return new VisionOutPuts(pitch, robotRelativeYaw, distanceToTarget * speedMultiplier);
    }

    public Optional<Measure<Angle>> getAngleToGP(){
        if(!inputs[CameraLocation.FRONT.ordinal()].hasTarget) return Optional.empty();

        return Optional.of(Units.Degrees.of(inputs[CameraLocation.FRONT.ordinal()].xAngleDegrees[0]));
    }


    @Override
    public void periodic() {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].update(inputs[i]);

            if (cameraLocations[i].HAS_POSE_ESTIMATION && inputs[i].hasTarget) {
                poseConsumer.accept(
                        new Pair<>(inputs[i].estimatedPose.toPose2d(), inputs[i].timestamp));
            }

            Logger.processInputs("Vision/" + cameraLocations[i].NAME, inputs[i]);
        }

    }


    public static class VisionOutPuts {
        private final Measure<Angle> pitch;
        private final Rotation2d yaw;
        private final Measure<Velocity<Angle>> velocity;

        /***
         * creates vision outputs object
         * @param pitch the pitch to the target in radians
         * @param yaw the yaw to the target in radians
         * @param velocity the velocity needed to shoot the gp in rpm
         */
        public VisionOutPuts(double pitch, double yaw, double velocity) {
            this.pitch = Units.Radians.of(pitch);
            this.yaw = Rotation2d.fromRadians(yaw);
            this.velocity = Units.RPM.of(velocity);
        }

        public Measure<Angle> getPitch() {
            return pitch;
        }

        public Rotation2d getYaw() {
            return yaw;
        }

        public Measure<Velocity<Angle>> getSpeed() {
            return velocity;
        }
    }
}
