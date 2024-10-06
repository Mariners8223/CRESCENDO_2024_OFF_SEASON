// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveTrain.DriveBaseConstants;

/**
 * Add your docs here.
 */
public class VisionConstants {

    public enum CameraLocation {
        FRONT_RIGHT(
            new Transform3d(-0.75/2 - 0.03, -0.75/2 + 0.06, -0.095 - DriveBaseConstants.CHASSIS_HEIGHT, new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(16))),
                "FrontRight",
                true
        ),
        BACK_LEFT(
                new Transform3d(-0.75/2 + 0.05, 0.75/2 - 
                0.14, -0.09 - DriveBaseConstants.CHASSIS_HEIGHT, new Rotation3d(0, Units.degreesToRadians(-32), Units.degreesToRadians(-180 + 16))),
                "BackLeft",
                true
        ),
        FRONT(
                new Transform3d(),
                "FrontCamera",
                false
        );

        public final Transform3d ROBOT_TO_CAMERA;

        public final String NAME;

        public final boolean HAS_POSE_ESTIMATION;

        CameraLocation(Transform3d cameraToRobot, String name, boolean hasPoseEstimation) {
            ROBOT_TO_CAMERA = cameraToRobot;
            NAME = name;
            HAS_POSE_ESTIMATION = hasPoseEstimation;
        }
    }

    public enum SpeakerConstants{
        RED(4),
        BLUE(7);

        public final int SPEAKER_CENTER_ID;

        public final Translation3d SPEAKER_CENTER_APRIL_TAG;
        public final Translation3d SPEAKER_TARGET;

        public final double SPEAKER_Y_START = 5.1;
        public final double SPEAKER_Y_END = 5.9;

        public final double ROBOT_Y_START = 2.5;
        public final double ROBOT_Y_END = 8;

        SpeakerConstants(int speakerCenterId){
            AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

            SPEAKER_CENTER_ID = speakerCenterId;

            SPEAKER_CENTER_APRIL_TAG = FIELD_LAYOUT.getTagPose(SPEAKER_CENTER_ID).isEmpty() ?
                    new Translation3d() :
                    FIELD_LAYOUT.getTagPose(SPEAKER_CENTER_ID).get().getTranslation();


            double TARGET_HEIGHT_ABOVE_APRIL_TAG = 0.8;

            SPEAKER_TARGET =
                    new Translation3d(SPEAKER_CENTER_APRIL_TAG.getX(), SPEAKER_CENTER_APRIL_TAG.getY(), SPEAKER_CENTER_APRIL_TAG.getZ() + TARGET_HEIGHT_ABOVE_APRIL_TAG);
        }
    }
}
