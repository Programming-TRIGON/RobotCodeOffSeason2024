package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose2d;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagLimelightIO;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagPhotonCameraIO;
import org.photonvision.simulation.SimCameraProperties;

import java.util.function.BiFunction;

public class AprilTagCameraConstants {
    static final double MAXIMUM_DISTANCE_FROM_TAG_FOR_SOLVE_PNP_METERS = 2.5;
    static final int CALCULATE_YAW_ITERATIONS = 3;
    static final Pose2d[] EMPTY_POSE_LIST = new Pose2d[0];

    public static final double MAXIMUM_AMBIGUITY = 0.5;

    public enum AprilTagCameraType {
        PHOTON_CAMERA(AprilTagPhotonCameraIO::new),
        LIMELIGHT((name, properties) -> new AprilTagLimelightIO(name));

        final BiFunction<String, SimCameraProperties, AprilTagCameraIO> createIOBiFunction;

        AprilTagCameraType(BiFunction<String, SimCameraProperties, AprilTagCameraIO> createIOBiFunction) {
            this.createIOBiFunction = createIOBiFunction;
        }
    }
}
