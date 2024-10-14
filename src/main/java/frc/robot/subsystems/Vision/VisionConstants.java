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

/**
 * Add your docs here.
 */
public class VisionConstants {

    public enum CameraLocation {
        FRONT_RIGHT(
             new Transform3d(new Translation3d(0.4  , -0.37, 0.17), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-17.3), Units.degreesToRadians(13))),
            new Transform3d(new Translation3d(0.4  , -0.37, 0.17), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-30), Units.degreesToRadians(13))),
                "FrontRight",
                true
        ),
        BACK_LEFT(
                new Transform3d(-0.29, 0.29, 0.19, new Rotation3d(Units.degreesToRadians(-1), Units.degreesToRadians(-14), Units.degreesToRadians(193))),
                new Transform3d(-0.29, 0.29, 0.19, new Rotation3d(Units.degreesToRadians(-1), Units.degreesToRadians(-14), Units.degreesToRadians(193))),
                "BackLeft",
                true
        ),
        FRONT(
                new Transform3d(),
                new Transform3d(),
                "FrontCamera",
                false
        );

        public final Transform3d ROBOT_TO_CAMERA;
        public final Transform3d CAMERA_OFFSETS_2D;

        public final String NAME;

        public final boolean HAS_POSE_ESTIMATION;

        CameraLocation(Transform3d robotToCamera, Transform3d TwodOffsets, String name, boolean hasPoseEstimation) {
            ROBOT_TO_CAMERA = robotToCamera;
            CAMERA_OFFSETS_2D = TwodOffsets;
            NAME = name;
            HAS_POSE_ESTIMATION = hasPoseEstimation;
        }
    }

    public static final double tolarance = 0.2;

    public enum PipeLineID{
        THREE_DIMENSIONAL,
        TWO_DIMENSIONAL
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


            double TARGET_HEIGHT_ABOVE_APRIL_TAG = 0.72;

            SPEAKER_TARGET =
                    new Translation3d(SPEAKER_CENTER_APRIL_TAG.getX(), SPEAKER_CENTER_APRIL_TAG.getY(), SPEAKER_CENTER_APRIL_TAG.getZ() + TARGET_HEIGHT_ABOVE_APRIL_TAG);
        }
    }
}
