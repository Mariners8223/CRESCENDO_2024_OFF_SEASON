// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.Camera;

import frc.robot.subsystems.Vision.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Add your docs here.
 */
public class LogitechIO implements CameraIO {

    private final PhotonCamera camera;


    public LogitechIO(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void setPipeline(VisionConstants.PipeLineID pipelineID) {
        
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

        inputs.xAngleDegrees[0] = result.getBestTarget().getYaw();
        inputs.yAngleDegrees[0] = result.getBestTarget().getPitch();
    }
}
