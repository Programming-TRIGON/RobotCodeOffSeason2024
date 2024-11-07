package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that contains commands that are used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static boolean IS_ALIGNING_TO_NOTE = false;

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
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.FINISHED_INTAKE_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command getFeedNoteCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING),
                GeneralCommands.getVisualizeNoteShootingCommand()
        ).withTimeout(AutonomousConstants.AUTONOMOUS_FEEDING_TIME_SECONDS);
    }

    public static Command getAlignToNoteCommand() {
        return new InstantCommand(
                () -> {
                    NOTE_DETECTION_CAMERA.startTrackingBestObject();
                    overrideRotation(AutonomousCommands::calculateRotationOverride);
                    IS_ALIGNING_TO_NOTE = true;
                }
        ).andThen(getSetCurrentLEDColorCommand());
    }

    public static Command getStopAligningToNoteCommand() {
        return new InstantCommand(
                () -> {
                    overrideRotation(Optional::empty);
                    IS_ALIGNING_TO_NOTE = false;
                }
        );
    }

    public static Command getPreparePitchForSpeakerShotCommand() {
        return new ParallelCommandGroup(
                ShootingCommands.getUpdateShootingCalculationsCommand(false),
                PitcherCommands.getReachTargetPitchFromShootingCalculationsCommand()
        );
    }

    public static Command getPreparePitchCommand(Rotation2d pitch) {
        return new ParallelCommandGroup(
                ShootingCommands.getUpdateShootingCalculationsCommand(false),
                PitcherCommands.getSetTargetPitchCommand(pitch)
        );
    }

    public static Command getPrepareShooterForSpeakerShotCommand() {
        return ShooterCommands.getReachTargetShootingVelocityFromShootingCalculationsCommand();
    }

    public static Command getPrepareShooterCommand(double targetVelocityRotationsPerSecond) {
        return ShooterCommands.getSetTargetVelocityCommand(targetVelocityRotationsPerSecond, targetVelocityRotationsPerSecond * ShooterConstants.RIGHT_MOTOR_TO_LEFT_MOTOR_RATIO);
    }

    private static Optional<Rotation2d> calculateRotationOverride() {
        NOTE_DETECTION_CAMERA.trackObject();
        if (RobotContainer.INTAKE.hasNote() || !NOTE_DETECTION_CAMERA.hasTargets())
            return Optional.empty();

        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        final Rotation2d targetRotation = NOTE_DETECTION_CAMERA.getTrackedObjectYaw().minus(currentRotation);
        return Optional.of(targetRotation);
    }

    private static void overrideRotation(Supplier<Optional<Rotation2d>> rotationOverride) {
        PPHolonomicDriveController.setRotationTargetOverride(rotationOverride);
    }

    private static Command getSetCurrentLEDColorCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(IntakeConstants.NOTE_DETECTION_CAMERA_HAS_TARGETS_BREATHING_LEDS_COLOR, LEDStrip.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(IntakeConstants.NOTE_DETECTION_CAMERA_HAS_NO_TARGETS_BREATHING_LEDS_COLOR, LEDStrip.LED_STRIPS),
                NOTE_DETECTION_CAMERA::hasTargets
        ).asProxy().until(() -> !IS_ALIGNING_TO_NOTE);
    }
}