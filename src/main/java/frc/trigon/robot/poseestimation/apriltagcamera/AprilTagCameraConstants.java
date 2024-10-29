package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose2d;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagLimelightIO;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagPhotonCameraIO;

import java.util.function.Function;

public class AprilTagCameraConstants {
    static final double MAXIMUM_DISTANCE_FROM_TAG_FOR_SOLVE_PNP_METERS = 2.5;
    static final int CALCULATE_YAW_ITERATIONS = 3;
    static final Pose2d[] EMPTY_POSE_LIST = new Pose2d[0];

    public static final double MAXIMUM_AMBIGUITY = 0.5;

    public enum AprilTagCameraType {
        PHOTON_CAMERA(AprilTagPhotonCameraIO::new),
        LIMELIGHT(AprilTagLimelightIO::new);

        final Function<String, AprilTagCameraIO> createIOFunction;

        AprilTagCameraType(Function<String, AprilTagCameraIO> createIOFunction) {
            this.createIOFunction = createIOFunction;
        }
    }
}
