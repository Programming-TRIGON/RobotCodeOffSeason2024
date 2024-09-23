package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that contains commands that are used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;

    public static Command getResetPoseToAutoPoseCommand(Supplier<String> pathName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    final Pose2d autoStartPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName.get());
                    RobotContainer.POSE_ESTIMATOR.resetPose(new MirrorablePose2d(autoStartPose, true).get());
                }
        ).ignoringDisable(true);
    }

    public static Command getNoteCollectionCommand() {
        return IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT, true).onlyIf(() -> !RobotContainer.INTAKE.hasNote());
    }

    public static Command getPrepareForShooterEjectionCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command getPrepareForCloseShooterEjectionCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command getFeedNoteCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING, true),
                GeneralCommands.getVisualizeNoteShootingCommand().onlyIf(MotorSubsystem::isExtensiveLoggingEnabled)
        ).until(() -> !RobotContainer.INTAKE.hasNote());
    }

    public static Command getAlignToNoteCommand() {
        return new StartEndCommand(
                () -> {
                    NOTE_DETECTION_CAMERA.startTrackingBestObject();
                    overrideRotation(AutonomousCommands::getRotationOverride);
                },
                () -> overrideRotation(Optional::empty)
        );
    }

    public static Command getStopAligningToNoteCommand() {
        return new InstantCommand(() -> overrideRotation(Optional::empty));
    }

    private static Optional<Rotation2d> getRotationOverride() {
        NOTE_DETECTION_CAMERA.trackObject();
        System.out.println("Tracking object");
        if (RobotContainer.INTAKE.hasNote())
            return Optional.empty();
        if (NOTE_DETECTION_CAMERA.hasTargets())
            return Optional.of(NOTE_DETECTION_CAMERA.getTrackedObjectYaw());
        System.out.println("No targets");
        return Optional.empty();
    }

    private static void overrideRotation(Supplier<Optional<Rotation2d>> rotationOverride) {
        PPHolonomicDriveController.setRotationTargetOverride(rotationOverride);
    }
}