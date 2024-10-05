package frc.trigon.robot.constants;

import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;

public class CameraConstants {
    public static final double
            TRANSLATIONS_STD_EXPONENT = 0.005,
            THETA_STD_EXPONENT = 0.01;

    public static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = new ObjectDetectionCamera("Collection Camera");
}
