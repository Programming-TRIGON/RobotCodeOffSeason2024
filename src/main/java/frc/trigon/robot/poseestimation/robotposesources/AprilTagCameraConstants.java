package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.poseestimation.photonposeestimator.PhotonPoseEstimator;

import java.util.HashMap;
import java.util.function.Function;

public class AprilTagCameraConstants {
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();
    static final PhotonPoseEstimator.PoseStrategy
            PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_HEADING;
    static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    static {
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
    }

    static final double MAXIMUM_AMBIGUITY = 0.2;
    static final double MAXIMUM_DISTANCE_FROM_TAG_FOR_PNP_METERS = 2;
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
