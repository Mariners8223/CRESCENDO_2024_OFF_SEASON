package frc.robot.subsystems.Vision.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.Vision.VisionConstants.PipeLineID;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ArduoCamIO implements CameraIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private VisionConstants.PipeLineID pipelineID = VisionConstants.PipeLineID.THREE_DIMENSIONAL;

    private final Supplier<Pose2d> referencePose;

    public ArduoCamIO(VisionConstants.CameraLocation cameraLocation, Supplier<Pose2d> referencePose) {
        camera = new PhotonCamera(cameraLocation.NAME);

        camera.setPipelineIndex(PipeLineID.THREE_DIMENSIONAL.ordinal());

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
    public void setPipeline(VisionConstants.PipeLineID pipeLineID) {
        if(this.pipelineID == pipeLineID) return;

        switch (pipeLineID) {
            case THREE_DIMENSIONAL:
                camera.setPipelineIndex(0);
                break;

            case TWO_DIMENSIONAL:
                camera.setPipelineIndex(1);
                break;
        }
        this.pipelineID = pipeLineID;
    }

    private PipeLineID[] pipeLineIDs = PipeLineID.values();

    @Override
    public void update(CameraInputsAutoLogged inputs) {

        inputs.isConnected = camera.isConnected();

        if(!inputs.isConnected){
            inputs.hasTarget = false;
            inputs.hasPose = false;
            return;
        }

        PhotonPipelineResult result = camera.getLatestResult();
        inputs.pipelineID = pipeLineIDs[camera.getPipelineIndex()].name();

        if (!result.hasTargets()) {
            inputs.hasTarget = false;
            inputs.hasPose = false;
            return;
        }

        inputs.timestamp = result.getTimestampSeconds();
        inputs.latency = result.getLatencyMillis() / 1000;
        switch (pipelineID) {
            case THREE_DIMENSIONAL:
                updateThreeDimensional(inputs, result
                );
                break;
            case TWO_DIMENSIONAL:
                updateTwoDimensional(inputs, result);
                break;
        }
    }


    private void updateThreeDimensional(CameraInputsAutoLogged inputs, PhotonPipelineResult result) {

        poseEstimator.setReferencePose(referencePose.get());

        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update();

        inputs.hasTarget = false;

        if(estimatedPose.isPresent()){
            if(result.getBestTarget().getPoseAmbiguity() <= VisionConstants.tolarance){
                inputs.estimatedPose = estimatedPose.get().estimatedPose;
                inputs.hasPose = true;
            }
            else inputs.hasPose = false;

            // Logger.recordOutput("pitch", inputs.estimatedPose.getRotation().getZ());
            // inputs.estimatedPose = new Pose3d(inputs.estimatedPose.getX(), inputs.estimatedPose.getY(), 0, inputs.estimatedPose.getRotation());
        }
        else {
            inputs.hasPose = false;
        }
    }

    private void updateTwoDimensional(CameraInputsAutoLogged inputs, PhotonPipelineResult result) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (int i = 0; i < 4 && i < targets.size(); i++) {
            var target = targets.get(i);

            inputs.poseAmbiguity[i] = target.getPoseAmbiguity();
            inputs.targetID[i] = target.getFiducialId();
            inputs.xAngleDegrees[i] = target.getYaw();
            inputs.yAngleDegrees[i] = target.getPitch();
        }

        inputs.hasTarget = true;
        inputs.hasPose = false;

    }
}
