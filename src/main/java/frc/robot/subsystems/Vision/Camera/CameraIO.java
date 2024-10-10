package frc.robot.subsystems.Vision.Camera;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.Vision.VisionConstants;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {

    @AutoLog
    class CameraInputs {
        public double latency = 0;
        public double timestamp = 0;
        public boolean hasTarget = false;

        public String pipelineID = "NONE";

        public double[] poseAmbiguity = new double[4];
        public int[] targetID = new int[4];

        public double[] xAngleDegrees = new double[4];
        public double[] yAngleDegrees = new double[4];

        public Pose3d estimatedPose;
    }

    void update(CameraInputsAutoLogged inputs);

    void setPipeline(VisionConstants.PipeLineID pipelineID);

}
