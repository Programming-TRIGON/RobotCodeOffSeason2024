package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import org.photonvision.simulation.SimCameraProperties;

public class CameraConstants {
    public static final double
            TRANSLATIONS_STD_EXPONENT = 0.02,
            THETA_STD_EXPONENT = 0.02;

    private static final Transform3d
            FRONT_CENTER_TO_CAMERA = new Transform3d(
            new Translation3d(0.325, 0.0465, 0.192),
            new Rotation3d(0, Units.degreesToRadians(-31), 0)
    ),
            REAR_CENTER_TO_CAMERA = new Transform3d(
                    new Translation3d(-0.325 + 0.00975, 0, 0.095),
                    new Rotation3d(Math.PI - Units.degreesToRadians(0), Units.degreesToRadians(-33), Math.PI + Units.degreesToRadians(0))
            );

    private static final int
            CAMERA_WIDTH = 1600,
            CAMERA_HEIGHT = 1200,
            CAMERA_FPS = 60,
            AVERAGE_CAMERA_LATENCY_MILLISECONDS = 35,
            CAMERA_LATENCY_STANDARD_DEVIATIONS_MILLISECONDS = 5,
            CAMERA_EXPOSURE_TIME_MILLISECONDS = 10;
    private static final Rotation2d CAMERA_FOV = Rotation2d.fromDegrees(90);
    private static final double
            AVERAGE_PIXEL_ERROR = 0.25,
            PIXEL_STANDARD_DEVIATIONS = 0.08;
    public static final boolean SHOULD_USE_CAMERA_SIMULATION = true;
    public static final SimCameraProperties SIMULATION_CAMERA_PROPERTIES = new SimCameraProperties();

    static {
        configureSimulationCameraProperties();
    }

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

    private static void configureSimulationCameraProperties() {
        SIMULATION_CAMERA_PROPERTIES.setCalibration(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOV);
        SIMULATION_CAMERA_PROPERTIES.setCalibError(AVERAGE_PIXEL_ERROR, PIXEL_STANDARD_DEVIATIONS);
        SIMULATION_CAMERA_PROPERTIES.setFPS(CAMERA_FPS);
        SIMULATION_CAMERA_PROPERTIES.setAvgLatencyMs(AVERAGE_CAMERA_LATENCY_MILLISECONDS);
        SIMULATION_CAMERA_PROPERTIES.setLatencyStdDevMs(CAMERA_LATENCY_STANDARD_DEVIATIONS_MILLISECONDS);
        SIMULATION_CAMERA_PROPERTIES.setExposureTimeMs(CAMERA_EXPOSURE_TIME_MILLISECONDS);
    }
}
