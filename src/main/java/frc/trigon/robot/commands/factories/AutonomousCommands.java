package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
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
        return IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT).onlyIf(() -> !RobotContainer.INTAKE.hasNote());
    }

    public static Command getFeedNoteCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING),
                GeneralCommands.getVisualizeNoteShootingCommand()
        ).until(() -> !RobotContainer.INTAKE.hasNote());
    }

    public static Command getAlignToNoteCommand() {
        return new InstantCommand(
                () -> {
                    NOTE_DETECTION_CAMERA.startTrackingBestObject();
                    overrideRotation(AutonomousCommands::calculateRotationOverride);
                }
        ).andThen(getSetCurrentLEDColorCommand());
    }

    public static Command getStopAligningToNoteCommand() {
        return new InstantCommand(() -> overrideRotation(Optional::empty));
    }

    public static Command getPrepareForShooterEjectionCommand(boolean isClose) {
        return isClose ? getPrepareForCloseShooterEjectionCommand() : getPrepareForShooterEjectionCommand();
    }

    private static Command getPrepareForShooterEjectionCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    private static Command getPrepareForCloseShooterEjectionCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND)
        );
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
                LEDStripCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS),
                NOTE_DETECTION_CAMERA::hasTargets
        ).asProxy().until(RobotContainer.INTAKE::hasNote);
    }
}