package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import org.trigon.utilities.mirrorable.MirrorablePose2d;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that contains commands that are used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static boolean
            SHOULD_ALIGN_TO_NOTE = false,
            SHOULD_ALIGN_TO_SPEAKER = false;

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

    public static Command getFeedNoteCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING, true),
                GeneralCommands.getVisualizeNoteShootingCommand().onlyIf(MotorSubsystem::isExtensiveLoggingEnabled)
        ).until(() -> !RobotContainer.INTAKE.hasNote());
    }

    public static Command getAlignToSpeakerCommand() {
        return new FunctionalCommand(
                () -> {
                },
                () -> SHOULD_ALIGN_TO_SPEAKER = true,
                (interrupted) -> SHOULD_ALIGN_TO_SPEAKER = false,
                () -> RobotContainer.SWERVE.atAngle(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle())
        );
//        return new FunctionalCommand(
//                () -> overrideRotation(() -> Optional.of(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get())),
//                () -> {
//                    System.out.println("target angle" + SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get());
//                    System.out.println(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation());
//                },
//                (interrupted) -> overrideRotation(Optional::empty),
//
//                () -> RobotContainer.SWERVE.atAngle(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle())
//        );
    }

    public static Command getAlignToNoteCommand() {
        return new FunctionalCommand(
                NOTE_DETECTION_CAMERA::startTrackingBestObject,
                () -> SHOULD_ALIGN_TO_NOTE = true,
                (interrupted) -> SHOULD_ALIGN_TO_NOTE = false,
                () -> RobotContainer.SWERVE.atAngle(new MirrorableRotation2d(NOTE_DETECTION_CAMERA.getTrackedObjectYaw(), false))
        );
//        return new FunctionalCommand(
//                () -> {
//                    NOTE_DETECTION_CAMERA.startTrackingBestObject();
//                    overrideRotation(
//                            () -> {
//                                NOTE_DETECTION_CAMERA.trackObject();
//                                if (NOTE_DETECTION_CAMERA.hasTargets())
//                                    return Optional.of(NOTE_DETECTION_CAMERA.getTrackedObjectYaw());
//                                return Optional.empty();
//                            }
//                    );
//                },
//                NOTE_DETECTION_CAMERA::trackObject,
//                (interrupted) -> overrideRotation(Optional::empty),
//                () -> RobotContainer.SWERVE.atAngle(new MirrorableRotation2d(NOTE_DETECTION_CAMERA.getTrackedObjectYaw(), false))
//        );
    }

//    private static void overrideRotation(Supplier<Optional<Rotation2d>> rotationOverride) {
//        PPHolonomicDriveController.setRotationTargetOverride(rotationOverride);
//    }

    public static Optional<Rotation2d> getRotationOverride() {
        System.out.println("current rotation" + RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation());
        System.out.println("note" + SHOULD_ALIGN_TO_NOTE);
        System.out.println("speaker" + SHOULD_ALIGN_TO_SPEAKER);

        if (SHOULD_ALIGN_TO_NOTE) {
            NOTE_DETECTION_CAMERA.trackObject();
            if (NOTE_DETECTION_CAMERA.hasTargets()) {
                System.out.println("Tracked object yaw: " + NOTE_DETECTION_CAMERA.getTrackedObjectYaw());
                return Optional.of(NOTE_DETECTION_CAMERA.getTrackedObjectYaw());
            }
        }
        if (SHOULD_ALIGN_TO_SPEAKER) {
            System.out.println("Speaker angle: " + SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get());
            return Optional.of(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get());
        }
        System.out.println("No rotation override");
        return Optional.empty();
    }
}