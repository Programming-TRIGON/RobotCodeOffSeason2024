package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;

public class CameraConstants {
    public static final double
            TRANSLATIONS_STD_EXPONENT = 0.02,
            THETA_STD_EXPONENT = 0.01;

    private static final Transform3d
            FRONT_CENTER_TO_CAMERA = new Transform3d(
            new Translation3d(0.0465, 0.325, 0.192),
            new Rotation3d(0, Units.degreesToRadians(-31), 0)
    ),
            REAR_CENTER_TO_CAMERA = new Transform3d(
                    new Translation3d(-0.325 + 0.00975, 0, 0.095),
                    new Rotation3d(Math.PI - Units.degreesToRadians(0), Units.degreesToRadians(-33), Math.PI + Units.degreesToRadians(0))
            );

    public static final AprilTagCamera
            FRONT_TAG_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "FrontTagCamera",
            FRONT_CENTER_TO_CAMERA,
            THETA_STD_EXPONENT,
            TRANSLATIONS_STD_EXPONENT
    ),
            REAR_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "RearTagCamera",
                    REAR_CENTER_TO_CAMERA,
                    THETA_STD_EXPONENT,
                    TRANSLATIONS_STD_EXPONENT
            );
    public static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = new ObjectDetectionCamera("NoteDetectionCamera");
}
