package frc.robot.subsystems.Vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.subsystems.Vision.VisionConstants;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ArduoCamIO implements CameraIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private final Supplier<Pose2d> referencePose;

    public ArduoCamIO(VisionConstants.CameraLocation cameraLocation, Supplier<Pose2d> referencePose) {
        camera = new PhotonCamera(cameraLocation.NAME);

        this.referencePose = referencePose;

        poseEstimator = createPoseEstimator(camera, cameraLocation.ROBOT_TO_CAMERA);
    }


    private static PhotonPoseEstimator createPoseEstimator(PhotonCamera camera, Transform3d robotToCamera) {

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

        PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        return poseEstimator;
    }


    @Override
    public void update(CameraInputsAutoLogged inputs) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            inputs.hasTarget = false;
            return;
        }
 
        inputs.hasTarget = true;
        inputs.timestamp = result.getTimestampSeconds();
        inputs.latency = result.getLatencyMillis() / 1000;

        poseEstimator.setReferencePose(referencePose.get());

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update();

        if(estimatedPose.isPresent()){
            inputs.estimatedPose = estimatedPose.get().estimatedPose;

            // Logger.recordOutput("pitch", inputs.estimatedPose.getRotation().getY());
            // inputs.estimatedPose = new Pose3d(inputs.estimatedPose.getX(), inputs.estimatedPose.getY(), 0, inputs.estimatedPose.getRotation());
        }



        List<PhotonTrackedTarget> targets = result.getTargets();

        for (int i = 0; i < 4 && i < targets.size(); i++) {
            var target = targets.get(i);

            inputs.poseAmbiguity[i] = target.getPoseAmbiguity();
            inputs.targetID[i] = target.getFiducialId();
            inputs.xAngleDegrees[i] = target.getYaw();
            inputs.yAngleDegrees[i] = target.getPitch();
        }

    }
}
