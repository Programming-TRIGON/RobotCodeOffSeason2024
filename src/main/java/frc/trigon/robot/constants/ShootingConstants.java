package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShootingConstants {
    public static final double G_FORCE = 9.80665;
    public static final double
            SPEAKER_SHOT_STANDING_VELOCITY_ROTATIONS_PER_SECOND = 45,
            DELIVERY_STANDING_VELOCITY_ROTATIONS_PER_SECOND = 35;

    public static final Pose3d ROBOT_RELATIVE_PITCHER_PIVOT_POINT = new Pose3d(0.2521, 0, 0.15545, new Rotation3d(0, 0, Math.PI));
    public static final Transform3d PITCHER_PIVOT_POINT_TO_NOTE_EXIT_POSITION = new Transform3d(0.4209, 0, -0.0165, new Rotation3d());
}
