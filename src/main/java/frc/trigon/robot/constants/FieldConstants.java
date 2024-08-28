package frc.trigon.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.trigon.utilities.mirrorable.MirrorablePose2d;
import org.trigon.utilities.mirrorable.MirrorableTranslation3d;

import java.util.HashMap;

public class FieldConstants {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIdToPoseMap();
    public static final double
            FIELD_LENGTH_METERS = 16.54175,
            FIELD_WIDTH_METERS = 8.21;
    private static final int SPEAKER_TAG_ID = 7;
    private static final Translation3d SPEAKER_TAG_TO_SPEAKER = new Translation3d(0.15, 0.0, 0.82);
    public static final MirrorableTranslation3d
            SPEAKER_TRANSLATION = new MirrorableTranslation3d(TAG_ID_TO_POSE.get(SPEAKER_TAG_ID).getTranslation().plus(SPEAKER_TAG_TO_SPEAKER), true),
            TARGET_DELIVERY_POSITION = new MirrorableTranslation3d(2.5, 7, 0, true);
    public static final MirrorablePose2d IN_FRONT_OF_AMP_POSE = new MirrorablePose2d(1.842, 8.204 - 0.405, Rotation2d.fromDegrees(-90), true);

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIdToPoseMap() {
        final HashMap<Integer, Pose3d> tagIdToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            tagIdToPose.put(aprilTag.ID, aprilTag.pose);
        return tagIdToPose;
    }
}
