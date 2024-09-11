package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShootingConstants {
    public static final double G_FORCE = 9.80665;
    public static final double
            SPEAKER_SHOT_STANDING_VELOCITY_ROTATIONS_PER_SECOND = 45,
            DELIVERY_STANDING_VELOCITY_ROTATIONS_PER_SECOND = 35;

    public static final double
            CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND = 45,
            AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND = 20,
            MANUAL_LOW_DELIVERY_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND = 45,
            EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND = 25;
    public static final Rotation2d
            CLOSE_SHOT_PITCH = Rotation2d.fromDegrees(57),
            AMP_PITCH = Rotation2d.fromDegrees(45),
            MANUAL_LOW_DELIVERY_PITCH = Rotation2d.fromDegrees(12),
            EJECT_FROM_SHOOTER_PITCH = Rotation2d.fromDegrees(10);


    public static final Pose3d ROBOT_RELATIVE_PITCHER_PIVOT_POINT = new Pose3d(0.2521, 0, 0.15545, new Rotation3d(0, 0, Math.PI));
    public static final Transform3d PITCHER_PIVOT_POINT_TO_NOTE_EXIT_POSITION = new Transform3d(0.4209, 0, -0.0165, new Rotation3d());
}
