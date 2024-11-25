package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShootingConstants {
    public static final double G_FORCE = 9.80665;
    public static final double
            SPEAKER_SHOT_STANDING_VELOCITY_ROTATIONS_PER_SECOND = 45,
            DELIVERY_STANDING_VELOCITY_ROTATIONS_PER_SECOND = 35,
            FINISHED_INTAKE_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND = -10,
            SPEAKER_SHOT_STANDING_EXTRA_VELOCITY_BUFFER_ROTATIONS_PER_SECOND = 3;

    public static final double
            CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND = 45,
            AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND = 37,
            MANUAL_LOW_DELIVERY_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND = 45,
            EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND = 10,
            CLOSE_EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND = 5;
    public static final Rotation2d
            CLOSE_SHOT_PITCH = Rotation2d.fromDegrees(57),
            PREPARE_AMP_PITCH = Rotation2d.fromDegrees(50),
            SHOOT_AMP_PITCH = Rotation2d.fromDegrees(44),
            HIGH_EJECT_PITCH = Rotation2d.fromDegrees(45),
            MANUAL_LOW_DELIVERY_PITCH = Rotation2d.fromDegrees(20),
            EJECT_FROM_SHOOTER_PITCH = Rotation2d.fromDegrees(20),
            CLOSE_EJECT_FROM_SHOOTER_PITCH = Rotation2d.fromDegrees(20);

    public static final Pose3d ROBOT_RELATIVE_PITCHER_PIVOT_POINT = new Pose3d(0.2521, 0, 0.15545, new Rotation3d(0, 0, Math.PI));
    public static final Transform3d PITCHER_PIVOT_POINT_TO_NOTE_EXIT_POSITION = new Transform3d(0.4209, 0, -0.0165, new Rotation3d());
}
