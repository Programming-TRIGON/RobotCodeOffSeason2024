package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagLimelightIO;
import frc.trigon.robot.poseestimation.apriltagcamera.io.AprilTagPhotonCameraIO;

import java.util.function.Function;

public class AprilTagCameraConstants {
    public static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    static final double MAXIMUM_DISTANCE_FROM_TAG_FOR_PNP_METERS = 2;
    static final int CALCULATE_YAW_ITERATIONS = 3;
    static final Pose2d[] EMPTY_POSE_LIST = new Pose2d[0];

    public enum RobotPoseSourceType {
        PHOTON_CAMERA(AprilTagPhotonCameraIO::new),
        LIMELIGHT(AprilTagLimelightIO::new);

        final Function<String, AprilTagCameraIO> createIOFunction;

        RobotPoseSourceType(Function<String, AprilTagCameraIO> createIOFunction) {
            this.createIOFunction = createIOFunction;
        }
    }
}
